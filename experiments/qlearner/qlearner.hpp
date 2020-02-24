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

    /**
     * Implementation of the Q-learning reinforcement learning algorithm.
     * Handles the creation of the Q and R matrices and updates the Q matrix with the following formula:
     * Q[state][action] = Q[state][action] + learningRate * (R[state][action] + discountFactor * Max(Q[state]) - Q[state][action]
     */
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

        /**
         * Creates the blank Q and R matrices and sets the hyper parameters.
         *
         * @param NUM_STATES The number of states.
         * @param NUM_ACTIONS The number of actions.
         * @param discountFactor Defines how much future rewards should matter in the current state.
         *  Can only be between 0 and 1.
         *  If it's close to 1 then the future reward matters,
         *  if it's close to 0 then the current reward.
         * @param learningRate Defines how much the agent should learn from the reinforcement.
         *  If it's close to 1 then the agent learns the most,
         *  if it's close to 0 then the agent relies more on its already acquired knowledge.
         */
        QLearner(const int NUM_STATES, const int NUM_ACTIONS, const double discountFactor = 0.8f,
                 const double learningRate = 0.6f) : NUM_STATES(NUM_STATES),
                                                     NUM_ACTIONS(NUM_ACTIONS),
                                                     discountFactor(discountFactor),
                                                     learningRate(learningRate) {

            assertm(discountFactor <= 1 && discountFactor >= 0, "The discount factor should be between 0 and 1!");
            assertm(learningRate <= 1 && learningRate >= 0, "The learning rate should be between 0 and 1!");
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

        double getDiscountFactor() const {
            return discountFactor;
        }

        void setDiscountFactor(double discountFactor) {
            QLearner::discountFactor = discountFactor;
        }


        /**
         * Initializes the reward matrix.
         * @param impossibleStateActions Contains a list of int-int tuples,
         *  where the first value is the row (State) and the second value the column (Action)
         * @param rewards Contains a list of int-int-double tuples,
         *  where the first value is the row (State), the second value the column (Action) and the third value the reward.
         */
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

        /**
         * Reinitializes the Q matrix.
         */
        void clearQ() {
            Q = std::vector<std::vector<double>>(NUM_STATES);
            for (int i = 0; i < NUM_STATES; ++i) {
                Q[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
            }
        }

        /**
         * Updates the value of the Q matrix.
         * If the learning rate is higher than 0.1, then the agent is in an exploration state, meaning that it acts
         * completely randomly. Otherwise the agent uses the Q matrix and chooses the action that its policy deems most beneficial.
         *
         * Formula for updating the Q matrix:
         *  Q[state][action] = Q[state][action] + learningRate * (R[state][action] + discountFactor * Max(Q[state]) - Q[state][action]
         *
         * @param state the current state
         * @param nextState the next state, the state to which taking the action in @param state leads to.
         * @return
         */
        int train(int state, int nextState) {
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
                                       + learningRate * (R[state][possibleAction] + discountFactor * nextStateMaxValue -
                                                         Q[state][possibleAction]);
            return possibleAction;
        }


        /**
         * Writes the Q matrix to a file.
         * @param fileName the file's name to where the Q matrix will be written to.
         */
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

        /**
         * Reads the Q matrix from a file.
         * @param fileName the file's name from where the Q matrix will be read from.
         */
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


