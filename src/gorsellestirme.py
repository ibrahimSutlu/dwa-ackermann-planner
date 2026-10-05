import numpy as np
import math
import matplotlib.pyplot as plt

# Hareket fonksiyonu
def motion(x, u, dt):
    x[2] += u[1] * dt #yaw
    x[0] += u[0] * math.cos(x[2]) * dt #x
    x[1] += u[0] * math.sin(x[2]) * dt #y
    x[3] = u[0] #hız
    x[4] = u[1] #açı
    return x

# Trajektori tahmini fonksiyonu
def predict_trajectory(x_init, v, y, predict_time, dt):
    x = np.array(x_init)
    trajectory = np.array([x])
    time = 0
    while time <= predict_time:
        x = motion(x.copy(), [v, y], dt)
        trajectory = np.vstack((trajectory, x))
        time += dt
    return trajectory

# Başlangıç durumu (x, y, yaw, hız, yaw_rate)
x_init = np.array([0, 0, 0, 0, 0])

# Konfigürasyon (örnek değerler)
predict_time = 10.0  # 3 saniyelik tahmin süresi
dt = 0.1  # Zaman adımı
v = 1.0  # Sabit hız (1 m/s)
y = 0.3  # Sabit dönüş açısı değişimi (0.3 rad/s)

# Trajektoriyi hesapla
trajectory = predict_trajectory(x_init, v, y, predict_time, dt)

# Görselleştirme
plt.figure(figsize=(6, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], "-g", label="Tahmin Edilen Rota")
plt.scatter(trajectory[0, 0], trajectory[0, 1], color="blue", label="Başlangıç Konumu")
plt.quiver(trajectory[:, 0], trajectory[:, 1], np.cos(trajectory[:, 2]), np.sin(trajectory[:, 2]), 
           scale=10, color="red", alpha=0.5, label="Robot Yönü")

plt.xlabel("X Konumu (m)")
plt.ylabel("Y Konumu (m)")
plt.title("Robotun Tahmini Hareket Rotası")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
