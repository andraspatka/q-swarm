#ifndef THREAD_SAFE_RAND
#define THREAD_SAFE_RAND

#include <random>

namespace ql {
    class ThreadSafeRandom {
    private:
        std::mt19937 gen;
        std::uniform_int_distribution<> distrib;
    public:
        // generated random number will be in the interval [0, 100]
        explicit ThreadSafeRandom(int minValue = 0, int maxValue = 100) {
            std::random_device r;
            std::seed_seq seed{150, 569};
            gen = std::mt19937(seed);
            distrib = std::uniform_int_distribution<>(minValue, maxValue - 1);
        }

        /**
         * Get a random integer number
         * @return integer number between minValue and maxValue
         */
        int getRandomNumber() {
            return distrib(gen);
        }
    };
}
#endif


