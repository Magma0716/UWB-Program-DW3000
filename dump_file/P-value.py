import scipy.stats as stats

# data
n = 1000
sample_mean = 0.0984       # 你的平均值
sample_sd = 0.0724         # 你的標準差
mu_0 = 0             # 目標基準值

se = sample_sd / (n ** 0.5)
t_stat = (sample_mean - mu_0) / se
p_value = stats.t.sf(abs(t_stat), df=n-1) * 2  # 雙尾檢定

print(f"SE: {se}")
print(f"t-value: {t_stat}")
print(f"p-value: {p_value}")
