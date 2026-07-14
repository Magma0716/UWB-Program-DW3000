import numpy as np
import pandas as pd
import scipy.stats as stats
import matplotlib.pyplot as plt

# 設定中文字型
plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']  # 微軟正黑體
plt.rcParams['axes.unicode_minus'] = False

# ================= 1. 輸入你的實驗數據 =================
# 請填入你實際的樣本數 (假設三組的樣本數相同，若不同可分開設定)
n_samples = 2000  

# 整理表格資料
categories = ['T1']

# 定位殘差的平均值與標準差
residual_means = [0.0854]
residual_stds  = [0.0164]

# 位移跳動的平均值與標準差
jitter_means = [0.0633]
jitter_stds  = [0.0336]

# ================= 2. 計算 95% 置信區間 (CI) =================
def calculate_ci(means, stds, n):
    ci_err = []
    for m, s in zip(means, stds):
        # 計算標準誤差 (Standard Error)
        se = s / np.sqrt(n)
        # 95% 信心水準下的 t 臨界值
        t_crit = stats.t.ppf(0.975, df=n-1)
        # 誤差範圍
        margin_of_error = t_crit * se
        ci_err.append(margin_of_error)
    return ci_err

res_ci_err = calculate_ci(residual_means, residual_stds, n_samples)
jit_ci_err = calculate_ci(jitter_means, jitter_stds, n_samples)

# ================= 3. 計算 P 值 (兩兩比較) =================
def get_p_values(means, stds, n):
    # 組1 vs 組2 (未加密 vs AES-CCM)
    _, p_12 = stats.ttest_ind_from_stats(means[0], stds[0], n, means[1], stds[1], n)
    # 組2 vs 組3 (AES-CCM vs 填充)
    _, p_23 = stats.ttest_ind_from_stats(means[1], stds[1], n, means[2], stds[2], n)
    # 組1 vs 組3 (未加密 vs 填充)
    _, p_13 = stats.ttest_ind_from_stats(means[0], stds[0], n, means[2], stds[2], n)
    return p_12, p_23, p_13
'''
p_res = get_p_values(residual_means, residual_stds, n_samples)
p_jit = get_p_values(jitter_means, jitter_stds, n_samples)

print("--- 定位殘差 P-value 結果 ---")
print(f"未加密 vs AES-CCM: p = {p_res[0]:.4e}")
print(f"AES-CCM vs 47B填充: p = {p_res[1]:.4e}")

print("\n--- 位移跳動 P-value 結果 ---")
print(f"未加密 vs AES-CCM: p = {p_jit[0]:.4e}")
print(f"AES-CCM vs 47B填充: p = {p_jit[1]:.4e}")
'''
# ================= 4. 繪製帶有 CI 誤差棒的長條圖 =================
x = np.arange(len(categories))
width = 0.15

fig, ax = plt.subplots(figsize=(10, 6))

# 畫出定位殘差與位移跳動的長條圖，yerr 帶入剛剛算好的置信區間半寬
rects1 = ax.bar(x - width/2, residual_means, width, yerr=res_ci_err, label='Residual', capsize=5, color="#4A81DA")
rects2 = ax.bar(x + width/2, jitter_means, width, yerr=jit_ci_err, label='Jitter', capsize=5, color="#36C756")

print('\nresidual')

print('lower bound:')
for i in range(len(residual_means)):
    print(f'{residual_means[i] - res_ci_err[i]:.4f}')
 
print('upper bound:')
for i in range(len(residual_means)):
    print(f'{residual_means[i] + res_ci_err[i]:.4f}')

print('\njitter')

print('lower bound:')
for i in range(len(jitter_means)):
    print(f'{jitter_means[i] - jit_ci_err[i]:.4f}')
 
print('upper bound:')
for i in range(len(jitter_means)):
    print(f'{jitter_means[i] + jit_ci_err[i]:.4f}')

# ----------------- 新增：標註數據標籤 -----------------
# fmt='%.4f' 代表呈現到小數點後四位
# padding=10 代表字體與誤差棒頂端保持 10 點的距離，避免重疊
ax.bar_label(rects1, padding=10, fmt='%.4f', fontsize=10)
ax.bar_label(rects2, padding=10, fmt='%.4f', fontsize=10)
# ---------------------------------------------------

ax.set_ylabel('Value (unit: m)', fontsize=12)
ax.set_xticks(x)
ax.set_xticklabels(categories, fontsize=11)
ax.legend(loc='upper left')
ax.grid(axis='y', linestyle='--', alpha=0.7)

# 稍微調高 y 軸的最大值上限，避免最右邊的數據標籤被圖表框線擋到
ax.set_ylim(0, max(max(residual_means), max(jitter_means)) * 1.25)

plt.tight_layout()
plt.show()