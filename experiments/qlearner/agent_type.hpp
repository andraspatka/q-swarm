#ifndef AGENT_TYPE_HELPER
#define AGENT_TYPE_HELPER

#include <string>
#include <iostream>

namespace ql {
    class AgentTypeHelper {
    public:
        enum AgentType {
            SUSCEPTIBLE,
            INFECTIOUS,
            REMOVED
        };

        static std::string ParseAgentTypeAsgetAgentTypeAsString() {
            switch (this->agentType) {
                case INFECTIOUS:
                    return "INFECTIOUS";
                case SUSCEPTIBLE:
                    return "SUSCEPTIBLE";
                case REMOVED:
                    return "REMOVED";
            }
        }

        static Stage ParseStageFromString(const std::string& stageString) {
            if (stageString == "train") return Stage::TRAIN;
            if (stageString == "exploit") return Stage::EXPLOIT;
            std::cerr << "Invalid Stage value: " << stageString;
            exit(1);
        }

        static std::string ParseStringFromStage(const Stage &stage) {
            if (stage == Stage::TRAIN) return "TRAIN";
            if (stage == Stage::EXPLOIT) return "EXPLOIT";
            std::cerr << "Invalid Stage value: " << stage;
            exit(1);
        }
    };
}
#endif