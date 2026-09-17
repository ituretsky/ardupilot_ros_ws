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
    required = ['секунда', 'широта', 'долгота', 'высота', 'заряд', 'режим', 'шаг']
    missing = [c for c in required if c not in df.columns]
    if missing:
        print(f"В CSV отсутствуют колонки: {missing}")
        sys.exit(1)

    # Определяем момент старта и посадки
    # Старт — первая строка, где state != INIT
    start_mask = df['шаг'] != 'INIT'
    if start_mask.any():
        start_idx = start_mask.idxmax()
        home_lat = df.loc[start_idx, 'широта']
        home_lon = df.loc[start_idx, 'долгота']
    else:
        home_lat = df['широта'].iloc[0]
        home_lon = df['долгота'].iloc[0]

    # Создаём фигуру с тремя графиками
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))

    # 1. Траектория (lon vs lat)
    ax = axes[0]
    ax.plot(df['долгота'], df['широта'], '-', linewidth=1.5, color='blue', label='Траектория')
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
    ax.plot(df['секунда'], df['высота'], '-', color='green')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Высота, м (над уровнем моря)')
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

    plt.tight_layout()
    plt.savefig(output_path, dpi=100)
    print(f"График сохранён: {output_path}")


if __name__ == '__main__':
    main()