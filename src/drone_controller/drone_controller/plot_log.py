#!/usr/bin/env python3
"""
Построение графиков по журналу полёта дрона.

Использование:
    ros2 run drone_controller plot_log <csv> [output.png]
    python3 plot_log.py <csv> [output.png]
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) < 2:
        print("Использование: plot_log <track.csv> [output.png]")
        print("Пример: plot_log ~/drone_logs/track.csv ~/drone_logs/plot.png")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"Файл не найден: {csv_path}")
        sys.exit(1)

    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        output_path = os.path.splitext(csv_path)[0] + "_plot.png"

    # Читаем CSV
    df = pd.read_csv(csv_path)

    # Проверяем, что все нужные колонки есть
    required = ['секунда', 'широта', 'долгота', 'высота', 'заряд', 'режим', 'шаг',
                'vN', 'vE', 'vU', 'eN', 'eE', 'eU']
    missing = [c for c in required if c not in df.columns]
    if missing:
        print(f"В CSV отсутствуют колонки: {missing}")
        sys.exit(1)

    # Определяем момент старта (первая строка с реальными координатами)
    valid_mask = df['широта'] != 0.0
    if valid_mask.any():
        start_idx = valid_mask.idxmax()
    else:
        start_idx = 0
        
    home_lat = df.loc[start_idx, 'широта']
    home_lon = df.loc[start_idx, 'долгота']
    home_alt = df.loc[start_idx, 'высота']

    # Создаём фигуру с пятью графиками
    fig, axes = plt.subplots(5, 1, figsize=(10, 16))

    # 1. Траектория (lon vs lat)
    ax = axes[0]
    df_flight = df.iloc[start_idx:]
    ax.plot(df_flight['долгота'], df_flight['широта'], '-', linewidth=1.5, color='blue', label='Траектория')
    
    ax.plot(home_lon, home_lat, 'go', markersize=12, label='Home (взлёт)')
    ax.plot(df['долгота'].iloc[-1], df['широта'].iloc[-1], 'rs', markersize=10, label='Финиш')
    ax.set_xlabel('Долгота')
    ax.set_ylabel('Широта')
    ax.set_title('Траектория полёта')
    ax.grid(True)
    ax.legend()
    ax.set_aspect('equal', adjustable='datalim')

    # 2. Высота от времени
    ax = axes[1]
    ax.plot(df_flight['секунда'], df_flight['высота'] - home_alt, '-', color='green')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Высота, м (от точки взлёта)')
    ax.set_title('Высота')
    ax.grid(True)

    # Отметим момент включения RTL или LAND
    rtl_mask = df['режим'] == 'RTL'
    if rtl_mask.any():
        t_rtl = df.loc[rtl_mask.idxmax(), 'секунда']
        ax.axvline(t_rtl, color='orange', linestyle='--', label='RTL')
    land_mask = df['режим'] == 'LAND'
    if land_mask.any():
        t_land = df.loc[land_mask.idxmax(), 'секунда']
        ax.axvline(t_land, color='red', linestyle='--', label='LAND')
    ax.legend()

    # 3. Заряд от времени
    ax = axes[2]
    ax.plot(df['секунда'], df['заряд'], '-', color='crimson')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Заряд, %')
    ax.set_title('Заряд батареи')
    ax.grid(True)
    ax.set_ylim(0, 105)

    # Отметим пороги
    ax.axhline(20, color='orange', linestyle=':', label='RTL (20%)')
    ax.axhline(5, color='red', linestyle=':', label='LAND (5%)')
    ax.legend()

    # 4. Скорости от времени
    ax = axes[3]
    ax.plot(df['секунда'], df['vN'], label='vN (север)', color='blue')
    ax.plot(df['секунда'], df['vE'], label='vE (восток)', color='green')
    ax.plot(df['секунда'], df['vU'], label='vU (вверх)', color='orange')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Скорость, м/с')
    ax.set_title('Скорости PID')
    ax.grid(True)
    ax.legend()

    # 5. Ошибки от времени
    ax = axes[4]
    ax.plot(df['секунда'], df['eN'], label='eN (север)', color='blue')
    ax.plot(df['секунда'], df['eE'], label='eE (восток)', color='green')
    ax.plot(df['секунда'], df['eU'], label='eU (вверх)', color='orange')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Ошибка, м')
    ax.set_title('Ошибки позиции')
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.savefig(output_path, dpi=100)
    print(f"График сохранён: {output_path}")


if __name__ == '__main__':
    main()
