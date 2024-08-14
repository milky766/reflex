#include <stdio.h>
#include <stdlib.h>

#define WINDOW_SIZE (100)
#define MAX_SAMPLES (1400)
#define NUM(fixed_array) ((int)(sizeof(fixed_array) / sizeof(fixed_array[0])))

// プロトタイプ宣言
void read_csv(const char *filename, float samples[], int *num_samples);
void write_csv(const char *filename, float samples[], int num_samples, float processed[], int num_processed);

void read_csv(const char *filename, float samples[], int *num_samples) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("ファイルを開けません");
        exit(EXIT_FAILURE);
    }
    
    char line[256];
    *num_samples = 0;
    
    // ヘッダー行をスキップ
    fgets(line, sizeof(line), file);
    
    while (fgets(line, sizeof(line), file) && *num_samples < MAX_SAMPLES) {
        int count;
        float value;
        if (sscanf(line, "%d,%f", &count, &value) == 2) {
            samples[(*num_samples)++] = value;
        }
    }
    
    fclose(file);
}

void write_csv(const char *filename, float samples[], int num_samples, float processed[], int num_processed) {
    FILE *file = fopen(filename, "w");
    if (!file) {
        perror("ファイルを開けません");
        exit(EXIT_FAILURE);
    }

    // ヘッダーを出力
    fprintf(file, "Count,OriginalData,MovingAverage\n");
    
    // 元のサンプルと移動平均を出力
    for (int i = 0; i < num_samples; i++) {
        if (i >= WINDOW_SIZE - 1 && i - WINDOW_SIZE + 1 < num_processed) {
            fprintf(file, "%d,%g,%g\n", i + 1, samples[i], processed[i - WINDOW_SIZE + 1]);
        } else {
            fprintf(file, "%d,%g,\n", i + 1, samples[i]);
        }
    }

    fclose(file);
}
int main(void) {
    float samples[MAX_SAMPLES];
    int num_samples;
    int i, j;
    float sum = 0.0;
    
    read_csv("data.csv", samples, &num_samples);
    
    if (num_samples < WINDOW_SIZE) {
        fprintf(stderr, "エラー: 移動平均を計算するのに十分なデータがありません。\n");
        return EXIT_FAILURE;
    }

    int num_processed = num_samples - WINDOW_SIZE + 1;
    float processed[num_processed];

    // 移動平均の計算
    for (i = WINDOW_SIZE - 1; i < num_samples; i++) {
        sum = 0.0;
        for (j = 0; j < WINDOW_SIZE; j++) {
            sum += samples[i - j];
        }
        processed[i - WINDOW_SIZE + 1] = sum / WINDOW_SIZE;
    }

// 結果を表示
for (i = 0; i < num_samples; i++) {
    if (i >= WINDOW_SIZE - 1) {
        printf("%d,%g,%g\n", i + 1, samples[i], processed[i - WINDOW_SIZE + 1]);
    } else {
        printf("%d,%g,\n", i + 1, samples[i]);
    }
}
}