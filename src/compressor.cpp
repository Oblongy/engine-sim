#include "../include/compressor.h"

#include "../include/intake.h"
#include "../include/exhaust_system.h"
#include "../include/constants.h"

#include <cmath>
#include <algorithm>

namespace {
    constexpr double P_ambient = units::pressure(1.0, units::atm);
    constexpr double T_ambient = units::celcius(25.0);
    constexpr double gamma_air = 1.4;

    // Shaft work (W) to raise a molar airflow (mol/s) from ambient to a
    // pressure ratio, treating compression as isothermal and scaling by the
    // compressor's isentropic efficiency.
    double compressionPower(double molarFlow, double pressureRatio, double efficiency) {
        if (pressureRatio <= 1.0 || molarFlow <= 0.0) return 0.0;
        return molarFlow * constants::R * T_ambient * std::log(pressureRatio)
            / std::fmax(efficiency, 0.05);
    }

    double clamp01(double v) {
        return std::fmin(std::fmax(v, 0.0), 1.0);
    }
}

Compressor::Compressor() {
    m_type = Type::Turbocharger;
    m_intake = nullptr;
    m_exhaust = nullptr;
    m_maxPressureRatio = 1.0;
    m_pressureRatio = 1.0;
    m_chargePressure = P_ambient;
    m_chargeTemperature = T_ambient;
    m_shaftSpeed = 0.0;
    m_driveTorque = 0.0;
    m_blowOffLevel = 0.0;
    m_whinePhase = 0.0;
    m_wastegateOpen = false;
    m_noiseState = 0x9E3779B9u;
}

Compressor::~Compressor() {
    /* void */
}

void Compressor::initialize(const Parameters &params, Intake *intake, ExhaustSystem *exhaust) {
    m_params = params;
    m_type = params.type;
    m_intake = intake;
    m_exhaust = exhaust;

    m_maxPressureRatio = (P_ambient + std::fmax(params.maxBoost, 0.0)) / P_ambient;
    m_pressureRatio = 1.0;
    m_chargePressure = P_ambient;
    m_chargeTemperature = T_ambient;
    m_shaftSpeed = 0.0;
    m_driveTorque = 0.0;
    m_blowOffLevel = 0.0;
    m_whinePhase = 0.0;
    m_wastegateOpen = false;
}

void Compressor::destroy() {
    m_intake = nullptr;
    m_exhaust = nullptr;
}

double Compressor::intakeMolarFlow(double dt) const {
    if (m_intake == nullptr || dt <= 0.0) return 0.0;
    return std::fmax(m_intake->m_flow, 0.0) / dt;
}

void Compressor::process(double dt, double crankSpeed, double throttleOpen) {
    if (m_intake == nullptr) return;

    throttleOpen = clamp01(throttleOpen);

    if (m_type == Type::Supercharger) {
        processSupercharger(dt, crankSpeed, throttleOpen);
    }
    else {
        processTurbocharger(dt, throttleOpen);
    }

    // The blow-off hiss dies away over a few hundred milliseconds.
    m_blowOffLevel *= std::exp(-dt / 0.3);
}

void Compressor::processSupercharger(double dt, double crankSpeed, double throttleOpen) {
    const double w = std::abs(crankSpeed);
    const double s = std::fmin(1.0, w / std::fmax(m_params.boostSpeed, 1.0));

    // Positive displacement blower: boost follows the square of its speed up to
    // the bypass setting. The bypass valve recirculates the blower output as the
    // throttle closes, so a closed throttle sees no boost.
    const double pressureRatio = 1.0 + (m_maxPressureRatio - 1.0) * s * s * throttleOpen;

    m_shaftSpeed = w * m_params.driveRatio;
    m_wastegateOpen = (s >= 1.0);

    applyCharge(pressureRatio);

    const double power = compressionPower(
        intakeMolarFlow(dt),
        m_pressureRatio,
        m_params.compressorEfficiency);

    m_driveTorque = m_params.parasiticTorque + power / std::fmax(w, units::rpm(500.0));
}

void Compressor::processTurbocharger(double dt, double throttleOpen) {
    // Turbine work from the exhaust collector: expansion from collector
    // pressure back to ambient, at the collector's temperature.
    double turbinePower = 0.0;
    if (m_exhaust != nullptr) {
        GasSystem *collector = m_exhaust->getSystem();
        const double P_exh = collector->pressure();
        const double T_exh = collector->temperature();
        const double molarFlow = std::abs(m_exhaust->getFlow()) / dt;

        if (P_exh > P_ambient && molarFlow > 0.0) {
            turbinePower = m_params.turbineEfficiency
                * molarFlow * constants::R * T_exh * std::log(P_exh / P_ambient);
        }
    }

    const double compressorPower = compressionPower(
        intakeMolarFlow(dt),
        m_pressureRatio,
        m_params.compressorEfficiency);

    // Shaft dynamics. The speed floor keeps power / speed finite at spool-up.
    const double w = std::fmax(m_shaftSpeed, units::rpm(2000.0));
    const double torque =
        (turbinePower - compressorPower) / w
        - m_params.bearingFriction * m_shaftSpeed;

    const double inertia = std::fmax(m_params.turbineInertia, 1e-9);
    m_shaftSpeed = std::fmax(0.0, m_shaftSpeed + (torque / inertia) * dt);

    // Wastegate: the shaft never runs past the speed that makes maxBoost.
    if (m_shaftSpeed >= m_params.maxSpeed) {
        m_shaftSpeed = m_params.maxSpeed;
        m_wastegateOpen = true;
    }
    else {
        m_wastegateOpen = false;
    }

    const double s = m_shaftSpeed / std::fmax(m_params.maxSpeed, 1.0);
    double pressureRatio = 1.0 + (m_maxPressureRatio - 1.0) * s * s;

    // Blow-off valve: throttle closes under boost, the charge vents to
    // atmosphere and the shaft freewheels.
    if (m_params.blowOffValve && throttleOpen < 0.15 && pressureRatio > 1.05) {
        const double level = (m_maxPressureRatio > 1.0)
            ? (pressureRatio - 1.0) / (m_maxPressureRatio - 1.0)
            : 0.0;
        m_blowOffLevel = std::fmax(m_blowOffLevel, level);
        pressureRatio = 1.0 + (pressureRatio - 1.0) * (throttleOpen / 0.15);
    }

    applyCharge(pressureRatio);
    m_driveTorque = 0.0;

    if (m_exhaust != nullptr) {
        m_exhaust->setOutletFlowScale(
            m_wastegateOpen ? m_params.wastegateRestriction : m_params.exhaustRestriction);
    }
}

void Compressor::applyCharge(double pressureRatio) {
    pressureRatio = std::fmin(std::fmax(pressureRatio, 1.0), m_maxPressureRatio);
    m_pressureRatio = pressureRatio;

    const double efficiency = std::fmax(m_params.compressorEfficiency, 0.05);
    const double T_out = T_ambient
        * (1.0 + (std::pow(pressureRatio, (gamma_air - 1.0) / gamma_air) - 1.0) / efficiency);

    m_chargeTemperature = T_out - clamp01(m_params.intercoolerEfficiency) * (T_out - T_ambient);
    m_chargePressure = P_ambient * pressureRatio;

    m_intake->setSource(m_chargePressure, m_chargeTemperature);
}

double Compressor::audioSample(double dt) {
    double out = 0.0;

    if (m_params.whineVolume > 0.0 && m_shaftSpeed > 0.0) {
        m_whinePhase += m_shaftSpeed * m_params.whineOrder * dt;
        if (m_whinePhase > 2.0 * constants::pi) {
            m_whinePhase = std::fmod(m_whinePhase, 2.0 * constants::pi);
        }

        const double reference = (m_type == Type::Turbocharger)
            ? m_params.maxSpeed
            : m_params.boostSpeed * m_params.driveRatio;
        const double s = std::fmin(m_shaftSpeed / std::fmax(reference, 1.0), 1.5);

        out += m_params.whineVolume * s * s * std::sin(m_whinePhase);
    }

    if (m_params.blowOffVolume > 0.0 && m_blowOffLevel > 1e-3) {
        // xorshift32 white noise
        m_noiseState ^= m_noiseState << 13;
        m_noiseState ^= m_noiseState >> 17;
        m_noiseState ^= m_noiseState << 5;
        const double noise = (m_noiseState / 4294967295.0) * 2.0 - 1.0;

        out += m_params.blowOffVolume * m_blowOffLevel * noise;
    }

    return out;
}
