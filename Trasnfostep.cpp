#include <stdio.h>
int stepsPerRev = 1000;

void Step(float angle1, float angle2, float angle3, int &step1, int &step2, int &step3) {
    step1 = angle1 / 360.0f * stepsPerRev;
    step2 = angle2 / 360.0f * stepsPerRev;
    step3 = angle3 / 360.0f * stepsPerRev;
}

void Stepper(const float angles[][3], int rows, int steps[][3]) {
    for (int i = 0; i < rows; ++i) {
        Step(angles[i][0], angles[i][1], angles[i][2],
             steps[i][0], steps[i][1], steps[i][2]);
    }
}

int main() {
    const float exampleAngles[3][3] = {
        {0.0f, 0.0f, 0.0f},
        {90.0f, 90.0f, 90.0f},
        {180.0f, 180.0f, 180.0f}
    };
    int exampleSteps[3][3] = {0};

    Stepper(exampleAngles, 3, exampleSteps);

    printf("Test Stepper:\n");
    printf("Angle1\tAngle2\tAngle3\tStep1\tStep2\tStep3\n");
    for (int i = 0; i < 3; ++i) {
        printf("%.1f\t%.1f\t%.1f\t%d\t%d\t%d\n",
               exampleAngles[i][0], exampleAngles[i][1], exampleAngles[i][2],
               exampleSteps[i][0], exampleSteps[i][1], exampleSteps[i][2]);
    }

    return 0;
}
