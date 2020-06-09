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
            RECOVERED,
            DECEASED
        };

        static std::string GetAgentTypeAsString(AgentType agentType) {
            switch (agentType) {
                case INFECTIOUS:
                    return "INFECTIOUS";
                case SUSCEPTIBLE:
                    return "SUSCEPTIBLE";
                case RECOVERED:
                    return "RECOVERED";
                case DECEASED:
                    return "DECEASED";
            }
        }
    };
}
#endif