#ifndef Q_LEARNER
#define Q_LEARNER

#include <vector>
#include <fstream>
#include <iostream>
#include <tuple>
#include <cassert>
#include "thread_safe_random.hpp"

#define assertm(exp, msg) assert(((void)msg, exp))

namespace ql {
    // TODO: write docs for this and for the methods
    // TODO: write getters and setters for the hyperparams so they can be tuned on the fly
    class QLearner {
    private:
        // hyperparameters
        double discountFactor;
        double learningRate;

        const int NUM_STATES;
        const int NUM_ACTIONS;

        std::vector<std::vector<double>> Q;
        std::vector<std::vector<double>> R;
    public:

        QLearner(const int NUM_STATES, const int NUM_ACTIONS, const double discountFactor = 0.8f,
                 const double learningRate = 0.6f, const double exploreExploit = 0.3f) : NUM_STATES(NUM_STATES),
                                                                                         NUM_ACTIONS(NUM_ACTIONS),
                                                                                         discountFactor(discountFactor),
                                                                                         learningRate(learningRate){
            Q = std::vector<std::vector<double>>(NUM_STATES);
            R = std::vector<std::vector<double>>(NUM_STATES);
            for (int i = 0; i < NUM_STATES; ++i) {
                Q[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
                R[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
            }
            srand(10);
        }

        double getLearningRate() const {
            return learningRate;
        }

        void setLearningRate(double learningRate) {
            this->learningRate = learningRate;
        }


        void initR(const std::vector<std::tuple<int, int>> &impossibleStateActions,
                   const std::vector<std::tuple<int, int, double>> &rewards) {
            for (auto impState : impossibleStateActions) {
                int stateIndex = std::get<0>(impState);
                int actionIndex = std::get<1>(impState);
                assertm(stateIndex < NUM_STATES, "Invalid state index!");
                assertm(actionIndex < NUM_ACTIONS, "Invalid action index!");
                R[stateIndex][actionIndex] = -1;
            }

            for (auto r : rewards) {
                int stateIndex = std::get<0>(r);
                int actionIndex = std::get<1>(r);
                double reward = std::get<2>(r);
                assertm(stateIndex < NUM_STATES, "Invalid state index!");
                assertm(actionIndex < NUM_ACTIONS, "Invalid action index!");
                assertm(reward > 0, "Reward value can not be negative!");
                R[stateIndex][actionIndex] = reward;
            }
        }

        void clearQ() {
            Q = std::vector<std::vector<double>>(NUM_STATES);
            for (int i = 0; i < NUM_STATES; ++i) {
                Q[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
            }
        }

        int explore(int state, int nextState) {
            ql::ThreadSafeRandom threadSafeRandom(0, NUM_ACTIONS);
            int possibleAction;
            do {
                possibleAction = threadSafeRandom.getRandomNumber();
            } while (R[state][possibleAction] == -1);
            double nextStateMaxValue = 0;
            for (int i = 0; i < NUM_ACTIONS; ++i) {
                nextStateMaxValue = std::max(nextStateMaxValue, Q[nextState][i]);
            }
            Q[state][possibleAction] = R[state][possibleAction] + discountFactor * nextStateMaxValue;
            return possibleAction;
        }

        int exploit(int state) {
            int actionWithMaxQ = 0;
            double maxQValue = 0;
            for (int i = 0; i < NUM_ACTIONS; ++i) {
                if (Q[state][i] > maxQValue) {
                    maxQValue = Q[state][i];
                    actionWithMaxQ = i;
                }
            }
            return actionWithMaxQ;
        }

        int exploreOrExploit(int state, int nextState) {
            int possibleAction = 0;
            if (learningRate > 0.1f) {

//                ql::ThreadSafeRandom threadSafeRandom(0, NUM_ACTIONS);
                do {
//                    possibleAction = threadSafeRandom.getRandomNumber();
                    possibleAction = rand() % 4;
                } while (R[state][possibleAction] == -1);
            } else {
                double maxQValue = 0;
                for (int i = 0; i < NUM_ACTIONS; ++i) {
                    if (Q[state][i] > maxQValue) {
                        maxQValue = Q[state][i];
                        possibleAction = i;
                    }
                }
            }

            double nextStateMaxValue = 0;
            for (int i = 0; i < NUM_ACTIONS; ++i) {
                nextStateMaxValue = std::max(nextStateMaxValue, Q[nextState][i]);
            }
            Q[state][possibleAction] = Q[state][possibleAction]
                    + learningRate * (R[state][possibleAction] + discountFactor * nextStateMaxValue - Q[state][possibleAction]);
            return possibleAction;
        }


        void printQ(const std::string &fileName) {
            std::ofstream file(fileName);
            if (!file.is_open()) {
                std::cout << "There was a problem opening the output file!\n";
                exit(1);
            }

            for (int i = 0; i < NUM_STATES; ++i) {
                for (int j = 0; j < NUM_ACTIONS; ++j) {
                    file << Q[i][j] << " ";
                }
                file << "\n";
            }
            file.close();
        }

        void readQ(const std::string &fileName) {
            std::ifstream file(fileName);
            if (!file.is_open()) {
                std::cerr << "There was a problem opening the input file!\n";
                exit(1);
            }

            for (int i = 0; i < NUM_STATES; ++i) {
                for (int j = 0; j < NUM_ACTIONS; ++j) {
                    file >> Q[i][j];
                }
            }
            file.close();
        }
    };
}
#endif


