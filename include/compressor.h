#ifndef ATG_ENGINE_SIM_COMPRESSOR_H
#define ATG_ENGINE_SIM_COMPRESSOR_H

#include "part.h"
#include "units.h"

class Intake;
class ExhaustSystem;

// Forced induction. A supercharger (crank driven) or a turbocharger (exhaust
// driven) feeding one intake. The compressor's only coupling into the gas model
// is the pressure and temperature of the charge air the intake draws from, so
// everything downstream (plenum, runners, chambers, exhaust, audio) responds to
// boost on its own.
class Compressor : public Part {
    public:
        enum class Type {
            Supercharger = 0,
            Turbocharger = 1
        };

        struct Parameters {
            Type type = Type::Turbocharger;

            // Boost above ambient at which the bypass valve (supercharger) or the
            // wastegate (turbocharger) holds the charge pressure.
            double maxBoost = units::pressure(8.0, units::psi);

            // Isentropic efficiency of the compressor. Sets the charge temperature
            // rise and the shaft work it takes to make boost.
            double compressorEfficiency = 0.7;

            // Fraction of the compressor's temperature rise an intercooler removes
            // again (0 = no intercooler, 1 = charge back at ambient).
            double intercoolerEfficiency = 0.0;

            // Supercharger: crank speed at which the blower reaches maxBoost. Boost
            // rises with the square of crank speed below it.
            double boostSpeed = units::rpm(4000.0);
            // Supercharger: blower speed / crank speed (whine pitch).
            double driveRatio = 2.5;
            // Supercharger: drag torque at the crank even with the bypass open.
            double parasiticTorque = units::torque(2.0, units::ft_lb);

            // Turbocharger: inertia of the rotating group (kg m^2). Sets the lag.
            double turbineInertia = 1e-5;
            // Turbocharger: shaft speed at which the compressor reaches maxBoost.
            double maxSpeed = units::rpm(120000.0);
            // Turbocharger: fraction of the exhaust's expansion work the turbine
            // turns into shaft work. Above 1 is a tuning fudge, not physics.
            double turbineEfficiency = 0.6;
            // Turbocharger: viscous bearing drag (N m s / rad).
            double bearingFriction = 1e-7;
            // Turbocharger: scale on the exhaust outlet flow while the turbine is
            // in the way (1 = free flowing). Lower = more back-pressure, quieter.
            double exhaustRestriction = 0.6;
            // Turbocharger: exhaust outlet flow scale once the wastegate is open.
            double wastegateRestriction = 1.0;
            // Turbocharger: vent the charge when the throttle closes under boost.
            bool blowOffValve = true;

            // Audio: whine amplitude (0 = silent) and the shaft order it plays at.
            // 1 = once per shaft revolution. Real blade counts put the tone above
            // what the synthesizer's input rate can carry, so keep it low.
            double whineVolume = 0.0;
            double whineOrder = 1.0;
            // Audio: blow-off hiss amplitude (0 = silent).
            double blowOffVolume = 0.0;
        };

    public:
        Compressor();
        virtual ~Compressor();

        void initialize(const Parameters &params, Intake *intake, ExhaustSystem *exhaust);
        virtual void destroy();

        // Advance one fluid step. crankSpeed is the output shaft's angular
        // velocity (rad/s, either sign); throttleOpen is 0 (closed) to 1 (wide open).
        void process(double dt, double crankSpeed, double throttleOpen);

        // One synthesizer input sample (whine + blow-off hiss) for this step.
        double audioSample(double dt);

        inline Type getType() const { return m_type; }
        inline Intake *getIntake() const { return m_intake; }
        inline ExhaustSystem *getExhaust() const { return m_exhaust; }
        inline double getChargePressure() const { return m_chargePressure; }
        inline double getChargeTemperature() const { return m_chargeTemperature; }
        inline double getPressureRatio() const { return m_pressureRatio; }
        inline double getBoost() const { return m_chargePressure - units::pressure(1.0, units::atm); }
        inline double getShaftSpeed() const { return m_shaftSpeed; }
        // Load the compressor puts on the crankshaft (supercharger only).
        inline double getDriveTorque() const { return m_driveTorque; }
        inline double getBlowOffLevel() const { return m_blowOffLevel; }
        inline bool isWastegateOpen() const { return m_wastegateOpen; }

    protected:
        void processSupercharger(double dt, double crankSpeed, double throttleOpen);
        void processTurbocharger(double dt, double throttleOpen);
        void applyCharge(double pressureRatio);
        double intakeMolarFlow(double dt) const;

        Type m_type;
        Parameters m_params;

        Intake *m_intake;
        ExhaustSystem *m_exhaust;

        double m_maxPressureRatio;
        double m_pressureRatio;
        double m_chargePressure;
        double m_chargeTemperature;
        double m_shaftSpeed;
        double m_driveTorque;
        double m_blowOffLevel;
        double m_whinePhase;
        bool m_wastegateOpen;
        unsigned int m_noiseState;
};

#endif /* ATG_ENGINE_SIM_COMPRESSOR_H */
