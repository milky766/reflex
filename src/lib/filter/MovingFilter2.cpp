#include <stdio.h>
#include <stdlib.h>

#define WINDOW_SIZE (10)
#define MAX_SAMPLES (1400)
#define NUM(fixed_array) ((int)(sizeof(fixed_array) / sizeof(fixed_array[0])))
#define ALPHA (2.0 / (WINDOW_SIZE + 1))

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
    fprintf(file, "Count,OriginalData,ExponentialMovingAverage\n");
    
    // 元のサンプルと指数移動平均を出力
    for (int i = 0; i < num_samples; i++) {
        if (i >= WINDOW_SIZE - 1 && i - WINDOW_SIZE + 1 < num_processed) {
            fprintf(file, "%d,%.6f,%.6f\n", i + 1, samples[i], processed[i - WINDOW_SIZE + 1]);
        } else {
            fprintf(file, "%d,%.6f,\n", i + 1, samples[i]);
        }
    }

    fclose(file);
}

int main(void) {
    float samples[MAX_SAMPLES];
    int num_samples;
    int i, j;
    
    read_csv("data.csv", samples, &num_samples);
    
    if (num_samples < WINDOW_SIZE) {
        fprintf(stderr, "エラー: 指数移動平均を計算するのに十分なデータがありません。\n");
        return EXIT_FAILURE;
    }

    int num_processed = num_samples - WINDOW_SIZE + 1;
    float processed[num_processed];
    
    // 指数移動平均の計算
    for (i = 0; i < num_processed; i++) {
        float ema = samples[i];
        for (j = 0; j < WINDOW_SIZE; j++) {
            ema = ALPHA * samples[i-j] + (1 - ALPHA) * ema;
        }
        processed[i] = ema;
    }

    // 結果を表示（デバッグ用）
    for (i = 0; i < num_processed; i++) {
        printf("%d,%.6f,%.6f\n", i + WINDOW_SIZE, samples[i+WINDOW_SIZE-1], processed[i]);
    }

    // 指数移動平均を新しいCSVファイルに書き込む
    write_csv("output2_10.csv", samples, num_samples, processed, num_processed);

    return 0;
}