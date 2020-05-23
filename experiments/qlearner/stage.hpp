#include <string>
#include <iostream>

namespace ql {
    class StageHelper {
    public:
        enum Stage {
            TRAIN,
            EXPLOIT
        };

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
