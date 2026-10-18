// Headless probe for forced induction. Compiles an engine script, holds the
// dyno at a series of engine speeds at full throttle and prints charge
// pressure, shaft speed, dyno torque and supercharger drive load.
//
//   boost-probe <main.mr> [rpm ...]
#include "../scripting/include/compiler.h"
#include "../include/units.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: boost-probe <main.mr> [rpm ...]\n");
        return 2;
    }

    std::vector<double> rpms;
    for (int i = 2; i < argc; ++i) rpms.push_back(std::atof(argv[i]));
    if (rpms.empty()) rpms = { 2000, 3000, 4000, 5000, 6000 };

    es_script::Compiler compiler;
    compiler.initialize();
    if (!compiler.compile(argv[1])) {
        std::fprintf(stderr, "compile failed (see es-script-compile-error.log)\n");
        return 1;
    }

    const es_script::Compiler::Output out = compiler.execute();
    compiler.destroy();
    if (out.engine == nullptr || out.vehicle == nullptr || out.transmission == nullptr) {
        std::fprintf(stderr, "script produced no engine\n");
        return 1;
    }

    Engine *engine = out.engine;
    Simulator *sim = engine->createSimulator(out.vehicle, out.transmission);
    engine->calculateDisplacement();
    sim->setSimulationFrequency(engine->getSimulationFrequency());

    // One-sample impulse so the synthesizer renders (and drains) without wave files.
    static int16_t impulse[1] = { 32767 };
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        sim->synthesizer().initializeImpulseResponse(impulse, 1, 1.0f, i);
    }
    sim->startAudioRenderingThread();

    std::printf("engine \"%s\"  %.2f L  compressors=%d\n",
        engine->getName().c_str(),
        engine->getDisplacement() / units::L,
        engine->getCompressorCount());

    engine->getIgnitionModule()->m_enabled = true;
    sim->m_starterMotor.m_enabled = true;

    static int16_t audio[4096];
    auto frame = [&]() {
        sim->startFrame(1.0 / 60.0);
        while (sim->simulateStep()) { /* run */ }
        sim->endFrame();
        sim->readAudioOutput(735, audio);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    };

    // Crank until it fires and idles.
    for (int i = 0; i < 60 * 6 && engine->getRpm() < 600.0; ++i) {
        engine->setSpeedControl(0.2);
        frame();
    }
    sim->m_starterMotor.m_enabled = false;
    std::printf("started: %.0f rpm\n", units::toRpm(engine->getSpeed()));

    sim->m_dyno.m_enabled = true;
    sim->m_dyno.m_hold = true;

    std::printf("%6s %6s %8s %7s %7s %8s %8s %6s\n",
        "rpm", "t(s)", "boost_psi", "ratio", "shaft_k", "chg_C", "dyno_ftlb", "drive");

    for (double target : rpms) {
        sim->m_dyno.m_rotationSpeed = units::rpm(target);
        for (int f = 0; f < 60 * 6; ++f) {
            engine->setSpeedControl(1.0);
            frame();

            if (f % 60 == 59) {
                double boost = 0, ratio = 1, shaft = 0, charge = 25, drive = 0;
                if (engine->getCompressorCount() > 0) {
                    Compressor *c = engine->getCompressor(0);
                    boost = c->getBoost() / units::psi;
                    ratio = c->getPressureRatio();
                    shaft = units::toRpm(c->getShaftSpeed()) / 1000.0;
                    charge = c->getChargeTemperature() - 273.15;
                    drive = c->getDriveTorque() / units::ft_lb;
                }
                std::printf("%6.0f %6.1f %8.2f %7.2f %7.1f %8.0f %8.1f %6.1f\n",
                    units::toRpm(engine->getSpeed()), (f + 1) / 60.0, boost, ratio,
                    shaft, charge, sim->getFilteredDynoTorque() / units::ft_lb, drive);
            }
        }
    }

    sim->endAudioRenderingThread();
    sim->destroy();
    return 0;
}
