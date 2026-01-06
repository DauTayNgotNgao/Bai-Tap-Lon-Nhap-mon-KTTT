#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

#define POP_SIZE 30
#define DIM 2
#define MAX_ITER 100
#define MIN_VAL -10.0
#define MAX_VAL 10.0

typedef struct {
    double position[DIM]; //x, y
    double fitness;//f(x)
} Whale;
//ham f(x,y) = x2 + y2
double sphere_function(double pos[DIM]) {
    double sum = 0.0;
    for (int i = 0; i < DIM; i++) {
        sum += pos[i] * pos[i];
    }
    return sum;
}

void copy_whale(Whale* dest, const Whale* src) {
    memcpy(dest->position, src->position, DIM * sizeof(double));
    dest->fitness = src->fitness;
}
//cap nhat con moi
void clamp_position(double pos[DIM]) {
    for (int i = 0; i < DIM; i++) {
        if (pos[i] < MIN_VAL) {
            pos[i] = MIN_VAL;
        }
        if (pos[i] > MAX_VAL) {
            pos[i] = MAX_VAL;
        }
    }
}

double rand_0_1() {
    return (double)rand() / RAND_MAX;
}

int main() {
    srand(time(NULL));

    Whale population[POP_SIZE];
    Whale best_whale;

    for (int i = 0; i < POP_SIZE; i++) {
        for (int j = 0; j < DIM; j++) {
            population[i].position[j] = MIN_VAL + (MAX_VAL - MIN_VAL) * rand_0_1();
        }
        population[i].fitness = sphere_function(population[i].position);

        if (i == 0 || population[i].fitness < best_whale.fitness) {
            copy_whale(&best_whale, &population[i]);
        }
    }

    printf("Khoi tao: Best Fitness = %e\n", best_whale.fitness);

    for (int iter = 0; iter < MAX_ITER; iter++) {
        double a = 2.0 - iter * (2.0 / MAX_ITER);
        for (int i = 0; i < POP_SIZE; i++) {
            double r1 = rand_0_1();
            double r2 = rand_0_1();
            double A = 2.0 * a * r1 - a;
            double C = 2.0 * r2;
            double p = rand_0_1();
            double l = (rand_0_1() * 2.0) - 1.0;
            double b = 1.0;

            for (int j = 0; j < DIM; j++) {
                if (p < 0.5) { //bao vay con moi
                    if (fabs(A) < 1.0) { //khai thac con moi hien tai
                        double D = fabs(C * best_whale.position[j] - population[i].position[j]);
                        population[i].position[j] = best_whale.position[j] - A * D;
                    } else { //boi ve 1 con ca voi ngau nhien khac
                        int rand_whale_idx = rand() % POP_SIZE;
                        Whale* rand_whale = &population[rand_whale_idx];
                        double D = fabs(C * rand_whale->position[j] - population[i].position[j]);
                        population[i].position[j] = rand_whale->position[j] - A * D;
                    }
                } else { //Tan Cong xoan oc
                    double D_prime = fabs(best_whale.position[j] - population[i].position[j]);
                    population[i].position[j] = D_prime * exp(b * l) * cos(2.0 * M_PI * l) + best_whale.position[j];
                }
            }

            clamp_position(population[i].position);
            //Tính fitness tại vị trí mới
            population[i].fitness = sphere_function(population[i].position);
            //cap nhat finess
            if (population[i].fitness < best_whale.fitness) {
                copy_whale(&best_whale, &population[i]);
            }
        }

        if ((iter + 1) % 10 == 0) {
            printf("Iteration %d, Best Fitness = %e\n", iter + 1, best_whale.fitness);
        }
    }

    printf("\n--- KET QUA CUOI CUNG ---\n");
    printf("Tim thay gia tri nho nhat tai:\n");
    printf("x = %f\n", best_whale.position[0]);
    printf("y = %f\n", best_whale.position[1]);
    printf("Gia tri fitness nho nhat (f(x,y)) = %e\n", best_whale.fitness);

    return 0;
}