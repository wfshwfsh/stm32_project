import re
import matplotlib.pyplot as plt
import pandas as pd

# 模擬LOG資料
with open("log2.txt") as f:
    log_data = f.read()

# 使用正則表達式擷取每行的數值
pattern = r"curAngle:(-?\d+\.\d+)\s+P:(-?\d+\.\d+)\s+I:(-?\d+\.\d+)\s+D:(-?\d+\.\d+)"
matches = re.findall(pattern, log_data)

# 轉為 DataFrame 並轉型為 float
df = pd.DataFrame(matches, columns=['curAngle', 'P', 'I', 'D']).astype(float)

# 新增一欄 PID 的總和
df['PID_sum'] = df['P'] + df['I'] + df['D']

# 畫圖
plt.figure(figsize=(10, 6))
plt.plot(df['curAngle'], label='curAngle', marker='o')
plt.plot(df['PID_sum'], label='PID_sum (P + I + D)', marker='x')
plt.title('curAngle and PID Sum Over Time')
plt.xlabel('Time (index)')
plt.ylabel('Value')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()