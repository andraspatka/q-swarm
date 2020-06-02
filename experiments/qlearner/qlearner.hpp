#ifndef Q_LEARNER
#define Q_LEARNER

#include <vector>
#include <fstream>
#include <iostream>
#include <tuple>
#include <cassert>
#include "thread_safe_random.hpp"
#include "state.hpp"
#include "action.hpp"

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
        double eGreedy;
    private:

        const int NUM_STATES;

        const int NUM_ACTIONS;
        std::vector<std::vector<double>> Q;

        std::vector<std::vector<double>> Q1;
        std::vector<std::vector<double>> Q2;
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
         * @param eGreedy Defines how much the agent should explore/exploit
         *  If it's close to 1 then the agent explores,
         *  if it's close to 0 then the agent exploits.
         */
        QLearner(const int NUM_STATES, const int NUM_ACTIONS, const double discountFactor = 0.8f,
                 const double learningRate = 0.6f, const double eGreedy = 0.1f) : NUM_STATES(NUM_STATES),
                                                     NUM_ACTIONS(NUM_ACTIONS),
                                                     discountFactor(discountFactor),
                                                     learningRate(learningRate),
                                                     eGreedy(eGreedy) {

            assertm(discountFactor <= 1 && discountFactor >= 0, "The discount factor should be between 0 and 1!");
            assertm(learningRate <= 1 && learningRate >= 0, "The learning rate should be between 0 and 1!");
            assertm(eGreedy <= 1 && eGreedy >= 0, "The explore-exploit ratio should be between 0 and 1!");
            Q = std::vector<std::vector<double>>(NUM_STATES);
            Q1 = std::vector<std::vector<double>>(NUM_STATES);
            Q2 = std::vector<std::vector<double>>(NUM_STATES);
            R = std::vector<std::vector<double>>(NUM_STATES);
            srand(12);
            for (int i = 0; i < NUM_STATES; ++i) {
                Q[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
                Q1[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
                Q2[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
                R[i] = std::vector<double>(NUM_ACTIONS, 0.0f);
            }
        }

        double getLearningRate() const {
            return learningRate;
        }

        void setLearningRate(double learningRate) {
            assertm(learningRate <= 1 && learningRate >= 0, "The learning rate should be between 0 and 1!");
            QLearner::learningRate = learningRate;
        }

        double getDiscountFactor() const {
            return discountFactor;
        }

        void setDiscountFactor(double discountFactor) {
            assertm(discountFactor <= 1 && discountFactor >= 0, "The discount factor should be between 0 and 1!");
            QLearner::discountFactor = discountFactor;
        }

        double getEGreedy() const {
            return eGreedy;
        }

        void setEGreedy(double eGreedy) {
            assertm(eGreedy <= 1 && eGreedy >= 0, "The explore-exploit ratio should be between 0 and 1!");
            QLearner::eGreedy = eGreedy;
        }


        /**
         * Initializes the reward matrix.
         * @param impossibleStateActions Contains a list of int-int tuples,
         *  where the first value is the row (State) and the second value the column (Action)
         * @param rewards Contains a list of int-int-double tuples,
         *  where the first value is the row (State), the second value the column (Action) and the third value the reward.
         */
        void initR(const std::vector<std::tuple<State, Action>> &impossibleStateActions,
                   const std::vector<std::tuple<State, Action, double>> &rewards) {
            for (auto impState : impossibleStateActions) {
                int stateIndex = std::get<0>(impState).getIndex();
                int actionIndex = std::get<1>(impState).getIndex();
                assertm(stateIndex < NUM_STATES && stateIndex >= 0, "Invalid state index!");
                assertm(actionIndex < NUM_ACTIONS && actionIndex >= 0, "Invalid action index!");
                R[stateIndex][actionIndex] = -1;
            }

            for (auto r : rewards) {
                int stateIndex = std::get<0>(r).getIndex();
                int actionIndex = std::get<1>(r).getIndex();
                double reward = std::get<2>(r);
                assertm(stateIndex < NUM_STATES && stateIndex >= 0, "Invalid state index!");
                assertm(actionIndex < NUM_ACTIONS && actionIndex >= 0, "Invalid action index!");
                assertm(reward > 0, "Reward value can not be negative!");
                R[stateIndex][actionIndex] = reward;
            }

            for (int i = 0; i < NUM_STATES; ++i) {
                for (int j = 0; j < NUM_ACTIONS; ++j) {
                    if (R[i][j] != -1) {
                        Q[i][j] = drand48();
                        Q1[i][j] = drand48();
                        Q2[i][j] = drand48();
                    } else {
                        Q[i][j] = -1;
                        Q1[i][j] = -1;
                        Q2[i][j] = -1;
                    }
                    if (i == NUM_STATES - 1) { // terminal state
                        Q[i][j] = 0;
                        Q1[i][j] = 0;
                        Q2[i][j] = 0;
                    }
                }
            }
        }

        /**
         * Updates the value of the Q matrix. Uses DoubleQ learning to avoid maximization bias (this results in overestimation).
         * Uses the epsilon-Greedy policy, the rate of exploration and exploitation determined by the epsilon hyperparameter.
         *
         * Formula for updating the Q matrix:
         *  Q1[state][action] = Q1[state][action] + learningRate * (R[state][action] + discountFactor * Max(Q2[state]) - Q1[state][action]
         *  Q2[state][action] = Q2[state][action] + learningRate * (R[state][action] + discountFactor * Max(Q1[state]) - Q2[state][action]
         *
         * @param state the current state
         * @param nextState the next state, the state to which taking the action in @param state leads to.
         * @return
         */
        Action doubleQ(const State& state, const State& nextState) {
            unsigned short stateIndex = state.getIndex();
            unsigned short nextStateIndex = nextState.getIndex();
            assertm(stateIndex < NUM_STATES && stateIndex >= 0, "Train: Invalid state index!");
            assertm(nextStateIndex < NUM_STATES && nextStateIndex >= 0, "Train: Invalid next state: index!");

            const double eGreedyRand = drand48();
            int actionIndex = 0;
            if (eGreedyRand < eGreedy) { // exploration
                do {
                    actionIndex = rand() % NUM_ACTIONS;
                } while (R[stateIndex][actionIndex] < 0);
            } else { // exploitation
                double maxActionValue = 0;
                for (int a = 0; a < NUM_ACTIONS; ++a) {
                    const double qsum = Q1[stateIndex][a] + Q1[stateIndex][a];
                    if (qsum > maxActionValue) {
                        maxActionValue = qsum;
                        actionIndex = a;
                    }
                }
            }
            const double qrand = drand48();

            std::vector<std::vector<double>> H;
            std::vector<std::vector<double>> Hi;

            bool isQ1 = qrand < 0.5f;
            H = isQ1 ? Q1 : Q2;
            Hi = isQ1 ? Q2 : Q1;

            int actionMaxH = 0;
            double valMaxH = 0;
            for (int a = 0; a < NUM_ACTIONS; ++a) {
                if (H[state.getIndex()][a] > valMaxH) {
                    valMaxH = H[state.getIndex()][a];
                    actionMaxH = a;
                }
            }

            H[stateIndex][actionIndex] = H[stateIndex][actionIndex] +
                    learningRate * (R[stateIndex][actionIndex] +
                    discountFactor * Hi[nextStateIndex][actionMaxH] - H[stateIndex][actionIndex]);

            Q1 = isQ1 ? H : Hi;
            Q2 = isQ1 ? Hi : H;

            return Action::fromIndex(actionIndex);
        }

        /**
         * Writes the Q matrix to a file.
         * @param fileName the file's name to where the Q matrix will be written to.
         */
        void printQ(const std::string &fileName, bool isDouble) {
            std::ofstream file(fileName);
            if (!file.is_open()) {
                std::cout << "There was a problem opening the output file!\n";
                exit(1);
            }

            for (int i = 0; i < NUM_STATES; ++i) {
                for (int j = 0; j < NUM_ACTIONS; ++j) {
                    if (isDouble) {
                        file << Q1[i][j] + Q2[i][j] << " ";
                    } else {
                        file << Q[i][j] << " ";
                    }
                }
                file << "\n";
            }
            file.close();
        }
    };
}
#endif


