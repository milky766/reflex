#include <iostream>
#include <deque>

class ExponentialMovingAverage {
public:
    // コンストラクタ
    ExponentialMovingAverage(int windowSize, double samplingRate)
        : windowSize(windowSize), alpha(2.0 / (windowSize + 1.0)), ema(0.0), isInitialized(false) {
        // ウィンドウサイズに基づいて平滑化係数を計算
        this->samplingRate = samplingRate;
    }

    // 新しいデータポイントを受け取り、EMAを更新
    void addDataPoint(double dataPoint) {
        if (!isInitialized) {
            // 初回のデータポイントでEMAを初期化
            ema = dataPoint;
            isInitialized = true;
        } else {
            // EMAの更新
            ema = alpha * dataPoint + (1 - alpha) * ema;
        }
        // 最新のデータポイントを保存
        dataPoints.push_back(dataPoint);
        if (dataPoints.size() > windowSize) {
            dataPoints.pop_front();
        }
    }

    // 現在のEMA値を取得
    double getEMA() const {
        return ema;
    }

private:
    int windowSize;      // ウィンドウサイズ
    double alpha;        // 平滑化係数
    double ema;          // 現在のEMA値
    bool isInitialized;  // 初期化フラグ
    double samplingRate; // サンプリングレート
    std::deque<double> dataPoints; // 最新のデータポイントを保持
};

int main() {
    int windowSize = 10; // 最新10データを利用
    double samplingRate = 100.0; // サンプリングレート 100Hz

    ExponentialMovingAverage ema(windowSize, samplingRate);

    // サンプルデータ
    double dataPoints[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110};
    int numDataPoints = sizeof(dataPoints) / sizeof(dataPoints[0]);

    // データポイントを追加してEMAを計算
    for (int i = 0; i < numDataPoints; ++i) {
        ema.addDataPoint(dataPoints[i]);
        std::cout << "New Data Point: " << dataPoints[i]
                  << ", Current EMA: " << ema.getEMA() << std::endl;
    }

    return 0;
}
