#ifndef ATG_ENGINE_SIM_COMPRESSOR_NODE_H
#define ATG_ENGINE_SIM_COMPRESSOR_NODE_H

#include "object_reference_node.h"

#include "engine_context.h"
#include "intake_node.h"
#include "exhaust_system_node.h"

#include "engine_sim.h"

namespace es_script {

    // Shared base for the supercharger and turbocharger script nodes. Both
    // produce a Compressor; they differ only in which parameters they expose.
    class CompressorNode : public ObjectReferenceNode<CompressorNode> {
    public:
        CompressorNode() { /* void */ }
        virtual ~CompressorNode() { /* void */ }

        Compressor *generate(Compressor *compressor, EngineContext *context) {
            Intake *intake = context->getIntake(m_intake);
            ExhaustSystem *exhaust = (m_exhaust == nullptr)
                ? nullptr
                : context->getExhaust(m_exhaust);

            compressor->initialize(m_parameters, intake, exhaust);

            return compressor;
        }

    protected:
        virtual void registerInputs() {
            addInput("intake", &m_intake, InputTarget::Type::Object);
            addInput("max_boost", &m_parameters.maxBoost);
            addInput("compressor_efficiency", &m_parameters.compressorEfficiency);
            addInput("intercooler_efficiency", &m_parameters.intercoolerEfficiency);
            addInput("whine_volume", &m_parameters.whineVolume);
            addInput("whine_order", &m_parameters.whineOrder);

            ObjectReferenceNode<CompressorNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        IntakeNode *m_intake = nullptr;
        ExhaustSystemNode *m_exhaust = nullptr;
        Compressor::Parameters m_parameters;
    };

    class SuperchargerNode : public CompressorNode {
    public:
        SuperchargerNode() { m_parameters.type = Compressor::Type::Supercharger; }
        virtual ~SuperchargerNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("boost_speed", &m_parameters.boostSpeed);
            addInput("drive_ratio", &m_parameters.driveRatio);
            addInput("parasitic_torque", &m_parameters.parasiticTorque);

            CompressorNode::registerInputs();
        }
    };

    class TurbochargerNode : public CompressorNode {
    public:
        TurbochargerNode() { m_parameters.type = Compressor::Type::Turbocharger; }
        virtual ~TurbochargerNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("exhaust_system", &m_exhaust, InputTarget::Type::Object);
            addInput("turbine_inertia", &m_parameters.turbineInertia);
            addInput("max_speed", &m_parameters.maxSpeed);
            addInput("turbine_efficiency", &m_parameters.turbineEfficiency);
            addInput("bearing_friction", &m_parameters.bearingFriction);
            addInput("exhaust_restriction", &m_parameters.exhaustRestriction);
            addInput("wastegate_restriction", &m_parameters.wastegateRestriction);
            addInput("blow_off_valve", &m_parameters.blowOffValve);
            addInput("blow_off_volume", &m_parameters.blowOffVolume);

            CompressorNode::registerInputs();
        }
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_COMPRESSOR_NODE_H */
