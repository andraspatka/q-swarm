#include <random>

class ThreadSafeRandom {
private:
    std::mt19937 gen;
    std::uniform_int_distribution<> distrib;
public:
    // generated random number will be in the intervall [0, 100]
    ThreadSafeRandom(int minValue = 0, int maxValue = 100) {
        std::random_device rd;
        gen = std::mt19937(rd());
        distrib = std::uniform_int_distribution<>(minValue, maxValue - 1);
    }

    int getRandomNumber() {
        return distrib(gen);
    }
};