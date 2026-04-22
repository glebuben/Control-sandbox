import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon, Circle, Rectangle
import warnings
warnings.filterwarnings('ignore')

# === Настройки стиля ===
plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.labelsize': 9,
    'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8,
    'figure.dpi': 120, 'savefig.dpi': 200, 'axes.grid': True, 'grid.alpha': 0.3
})

# === Параметры MRAC ===
a_m, b_m = 2.5, 1.5
gamma1, gamma2, sigma = 20.0, 20.0, 0.1

# === Сигналы и модель ===
def r(t): return 5.0*np.sin(0.6*t) + 2.0*np.sin(2.5*t) + 1.5
def aero_params(t):
    sig = 1.0 / (1.0 + np.exp(-0.8*(t - 8.0)))
    return -1.8 + 1.3*sig, 28.0 - 19.0*sig

def dynamics(t, x):
    q, qm, th1, th2 = x
    rt, e = r(t), x[0] - x[1]
    Mq, Mde = aero_params(t)
    delta_e = th1*rt + th2*x[0]
    return [Mq*x[0] + Mde*delta_e, -a_m*x[1] + b_m*rt,
            -gamma1*e*rt - sigma*gamma1*th1, -gamma2*e*x[0] - sigma*gamma2*th2]

# === Интегрирование ===
sol = solve_ivp(dynamics, (0, 20), [0,0,0,0], t_eval=np.linspace(0,20,400), method='RK45')
t, q, qm, th1, th2 = sol.t, sol.y[0], sol.y[1], sol.y[2], sol.y[3]
e, delta_e = q - qm, th1*np.array([r(tt) for tt in t]) + th2*q

# Интегрируем угловую скорость → угол тангажа (в радианах)
theta = np.cumsum(q) * (t[1]-t[0])
theta_m = np.cumsum(qm) * (t[1]-t[0])

# === Функция отрисовки силуэта F-16 (упрощённо) ===
def get_f16_shape(scale=1.0):
    """Возвращает точки силуэта F-16 в локальных координатах"""
    # Упрощённый контур: фюзеляж, крыло, стабилизатор
    fuselage = np.array([[-0.8,0], [-0.3,0.05], [0.3,0.08], [0.9,0], [0.3,-0.08], [-0.3,-0.05], [-0.8,0]])
    wing = np.array([[-0.2,0], [-0.1,0.3], [0.1,0.3], [0.2,0], [0.1,-0.02], [-0.1,-0.02]])
    stabilizer = np.array([[0.7,0], [0.6,0.12], [0.8,0.12], [0.9,0], [0.8,-0.12], [0.6,-0.12]])
    # Объединяем и масштабируем
    shape = np.vstack([fuselage*1.2, wing*scale*0.9, stabilizer*scale*0.5])
    return shape * scale

# === Инициализация анимации ===
fig = plt.figure(figsize=(14, 8))
fig.suptitle('✈️ F-16 Pitch-Rate: Direct MRAC — Адаптивная стабилизация в полёте', fontsize=13, fontweight='bold')

# --- Левая часть: анимация полёта ---
ax_flight = plt.subplot2grid((2, 3), (0, 0), rowspan=2, colspan=2)
ax_flight.set_xlim(-2, 12)
ax_flight.set_ylim(-3, 5)
ax_flight.set_aspect('equal')
ax_flight.set_xlabel('Положение по горизонтали [км]')
ax_flight.set_ylabel('Высота [км]')
ax_flight.set_title('A) Визуализация полёта')

# Фон: небо и земля
ax_flight.axhspan(-3, 0, color='lightgreen', alpha=0.2, label='Земля')
ax_flight.axhspan(0, 5, color='skyblue', alpha=0.3, label='Небо')
ax_flight.axhline(0, color='green', linewidth=1, linestyle='--', alpha=0.5)  # горизонт

# Элементы самолёта
f16_shape = get_f16_shape(scale=0.8)
aircraft = Polygon(f16_shape, closed=True, fill=True, color='gray', edgecolor='black', linewidth=1.5, zorder=5)
ax_flight.add_patch(aircraft)

# Точка цели (эталонная траектория) с опережением
target_dot, = ax_flight.plot([], [], 'ro', markersize=6, label='Эталонная траектория', zorder=4)
# След полёта
trail_actual, = ax_flight.plot([], [], 'b-', linewidth=0.8, alpha=0.6, label='Фактическая траектория')
trail_ref, = ax_flight.plot([], [], 'r--', linewidth=0.8, alpha=0.4)

# Вектор скорости
velocity_arrow = ax_flight.arrow(0, 0, 0, 0, head_width=0.15, head_length=0.3, 
                                fc='blue', ec='blue', linewidth=1.5, zorder=6, alpha=0.8)

# Текст статуса
status_text = ax_flight.text(0.02, 0.98, '', transform=ax_flight.transAxes, fontsize=9,
                            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6))

# --- Правая часть: графики в реальном времени ---
# График 1: Угол тангажа
ax_theta = plt.subplot2grid((2, 3), (0, 2))
line_theta_act, = ax_theta.plot([], [], 'b-', linewidth=1.5, label='θ(t) факт.')
line_theta_ref, = ax_theta.plot([], [], 'r--', linewidth=1.2, label='θ_m(t) эталон')
ax_theta.set_ylabel('Угол тангажа [°]')
ax_theta.set_title('B) Тангаж')
ax_theta.legend(fontsize=7)
ax_theta.grid(True, alpha=0.3)

# График 2: Ошибка и управление
ax_err = plt.subplot2grid((2, 3), (1, 2))
line_err, = ax_err.plot([], [], 'k-', linewidth=1, label='Ошибка e(t)')
ax_err.axhline(0, color='gray', linestyle=':', alpha=0.4)
ax_err.set_ylabel('Ошибка [°/с]', color='k')
ax_err.tick_params(axis='y', labelcolor='k')
ax_err.set_xlabel('Время [с]')
ax_err.set_title('C) Метрики')
ax_err.grid(True, alpha=0.3)

# Вторая ось для управления
ax_ctrl = ax_err.twinx()
line_ctrl, = ax_ctrl.plot([], [], 'm-', linewidth=1, label='δₑ (управление)')
ax_ctrl.set_ylabel('Руль высоты [°]', color='m')
ax_ctrl.tick_params(axis='y', labelcolor='m')
ax_ctrl.set_ylim(-30, 30)

# === Функция обновления кадра ===
def animate(frame):
    idx = frame % len(t)
    tt, th_act, th_ref = t[idx], np.degrees(theta[idx]), np.degrees(theta_m[idx])
    q_act, q_ref = np.degrees(q[idx]), np.degrees(qm[idx])
    err_val, ctrl_val = np.degrees(e[idx]), np.degrees(delta_e[idx])
    
    # Позиция самолёта (интегрируем горизонтальную скорость ~ 250 м/с = 0.25 км/с)
    x_pos = 0.25 * tt  # км
    y_pos = 0.5 + 0.3 * np.sin(0.2*tt) + 0.02 * th_act  # км, с небольшой коррекцией по тангажу
    
    # Обновление силуэта: поворот и перемещение
    angle_rad = np.radians(th_act)
    rotated = np.array([
        f16_shape[:,0]*np.cos(angle_rad) - f16_shape[:,1]*np.sin(angle_rad) + x_pos,
        f16_shape[:,0]*np.sin(angle_rad) + f16_shape[:,1]*np.cos(angle_rad) + y_pos
    ]).T
    aircraft.set_xy(rotated)
    
    # Вектор скорости (направлен по тангажу)
    v_len = 1.2
    ax_flight.arrow(x_pos, y_pos, 
                   v_len*np.cos(angle_rad), v_len*np.sin(angle_rad),
                   head_width=0.12, head_length=0.25, fc='blue', ec='blue', 
                   linewidth=1.5, zorder=6, alpha=0.7)
    
    # Эталонная точка (с небольшим опережением по времени для наглядности)
    target_idx = min(idx + 15, len(t)-1)
    x_target = 0.25 * t[target_idx]
    y_target = 0.5 + 0.3*np.sin(0.2*t[target_idx]) + 0.02*np.degrees(theta_m[target_idx])
    target_dot.set_data([x_target], [y_target])
    
    # Следы траекторий
    trail_len = max(0, idx-50)
    trail_actual.set_data([0.25*t[i] for i in range(trail_len, idx+1)],
                         [0.5+0.3*np.sin(0.2*t[i])+0.02*np.degrees(theta[i]) for i in range(trail_len, idx+1)])
    trail_ref.set_data([0.25*t[i] for i in range(trail_len, min(idx+16, len(t)))],
                      [0.5+0.3*np.sin(0.2*t[i])+0.02*np.degrees(theta_m[i]) for i in range(trail_len, min(idx+16, len(t)))])
    
    # Статус
    phase = "🚀 Набор / Маневр" if tt < 12 else "✅ Стабилизация"
    err_status = "✓ Ошибка < 1°/с" if np.abs(err_val) < 1 else "⚠ Коррекция"
    status_text.set_text(f'Время: {tt:.1f} с\n{phase}\n{err_status}\nθ: {th_act:.1f}° | δₑ: {ctrl_val:.1f}°')
    
    # Обновление графиков справа
    # Угол тангажа
    line_theta_act.set_data(t[:idx+1], np.degrees(theta[:idx+1]))
    line_theta_ref.set_data(t[:idx+1], np.degrees(theta_m[:idx+1]))
    ax_theta.set_xlim(0, 20)
    ax_theta.set_ylim(-20, 20)
    
    # Ошибка и управление
    line_err.set_data(t[:idx+1], np.degrees(e[:idx+1]))
    line_ctrl.set_data(t[:idx+1], np.degrees(delta_e[:idx+1]))
    ax_err.set_xlim(0, 20)
    ax_err.set_ylim(-5, 5)
    
    return aircraft, target_dot, trail_actual, trail_ref, velocity_arrow, status_text, line_theta_act, line_theta_ref, line_err, line_ctrl

# === Создание и запуск анимации ===
ani = animation.FuncAnimation(fig, animate, frames=len(t), interval=50, blit=False, repeat=True)

# Легенда для полёта
ax_flight.legend(loc='lower right', fontsize=8, framealpha=0.8)

# Кнопки управления
from matplotlib.widgets import Button
ax_pause = plt.axes([0.72, 0.02, 0.08, 0.04])
ax_save = plt.axes([0.82, 0.02, 0.08, 0.04])
btn_pause = Button(ax_pause, '⏸ Pause')
btn_save = Button(ax_save, '💾 Save GIF')

def pause_anim(event):
    if ani.event_source.is_running():
        ani.event_source.stop()
        btn_pause.label.set_text('▶ Play')
    else:
        ani.event_source.start()
        btn_pause.label.set_text('⏸ Pause')
btn_pause.on_clicked(pause_anim)

def save_anim(event):
    print("🎬 Сохранение анимации... (может занять ~30 сек)")
    ani.save('f16_mrac_flight.gif', writer='pillow', fps=30, dpi=100)
    print("✅ Сохранено: f16_mrac_flight.gif")
btn_save.on_clicked(save_anim)

plt.tight_layout(rect=[0, 0.06, 1, 0.96])
print("▶ Запуск анимации... Нажмите 'Pause' для остановки, 'Save GIF' для экспорта.")
print("💡 Совет: Увеличьте окно для лучшей видимости.")
plt.show()