"""
Скрипт для построения графика траектории по координатам из лог-файла.
Читает файл с координатами в формате словарей Python и строит график x-y.
Поддерживает отрисовку траекторий одного или нескольких дронов.
"""
import argparse
import ast
import matplotlib.pyplot as plt
import os

def parse_log_file(log_file):
    """
    Парсинг лог-файла с координатами.
    
    Args:
        log_file: путь к файлу с логами
    
    Returns:
        Списки координат x и y
    """
    if not os.path.exists(log_file):
        print(f"Error: File {log_file} not found!")
        return None, None
    
    x_coords = []
    y_coords = []
    
    print(f"Reading log file: {log_file}")
    
    with open(log_file, 'r') as f:
        line_count = 0
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            try:
                # Парсим строку как словарь Python
                coord_dict = ast.literal_eval(line)
                
                # Извлекаем координаты x и y
                if 'x' in coord_dict and 'y' in coord_dict:
                    x_coords.append(coord_dict['x'])
                    y_coords.append(-coord_dict['y'])
                    line_count += 1
                else:
                    print(f"Warning: Line {line_count + 1} missing 'x' or 'y' key")
                    
            except (ValueError, SyntaxError) as e:
                print(f"Warning: Could not parse line {line_count + 1}: {e}")
                continue
    
    print(f"Parsed {len(x_coords)} coordinate points")
    
    if len(x_coords) == 0:
        print("Error: No valid coordinates found in file!")
        return None, None
    
    return x_coords, y_coords

def calculate_distance(x_coords, y_coords):
    """Вычисляет общее пройденное расстояние."""
    if len(x_coords) < 2:
        return 0.0
    total_distance = 0
    for i in range(1, len(x_coords)):
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        total_distance += (dx**2 + dy**2)**0.5
    return total_distance

def plot_trajectory(x_coords, y_coords, title="Drone Trajectory", save_path=None, 
                    label="Trajectory", color='blue', show_arrows=True):
    """
    Построение графика траектории одного дрона.
    
    Args:
        x_coords: список координат x
        y_coords: список координат y
        title: заголовок графика (используется только для одного дрона)
        save_path: путь для сохранения (опционально)
        label: метка для легенды
        color: цвет траектории
        show_arrows: показывать ли стрелки направления
    """
    # Строим траекторию
    plt.plot(x_coords, y_coords, '-', linewidth=1.5, alpha=0.7, 
             label=label, color=color)
    
    # Отмечаем начальную точку
    if len(x_coords) > 0:
        plt.scatter(x_coords[0], y_coords[0], color='green', s=150, 
                   marker='o', zorder=5, edgecolors='black', linewidths=2)
    
    # Отмечаем конечную точку
    if len(x_coords) > 1:
        plt.scatter(x_coords[-1], y_coords[-1], color='red', s=150, 
                   marker='s', zorder=5, edgecolors='black', linewidths=2)
    
    # Добавляем стрелки для направления движения
    if show_arrows and len(x_coords) > 100:
        step = max(1, len(x_coords) // 20)  # 20 стрелок на графике
        for i in range(0, len(x_coords) - step, step):
            dx = x_coords[i + step] - x_coords[i]
            dy = y_coords[i + step] - y_coords[i]
            # Пропускаем стрелки, если движение слишком малое
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                plt.arrow(x_coords[i], y_coords[i], dx * 0.3, dy * 0.3,
                         head_width=0.5, head_length=0.5, fc=color, ec=color, 
                         alpha=0.4, length_includes_head=True)

def plot_multiple_trajectories(trajectories_data, title="Multiple Drones Trajectories", 
                               save_path=None, show_info=True):
    """
    Построение графика траекторий нескольких дронов.
    
    Args:
        trajectories_data: список словарей с ключами 'x', 'y', 'label', 'color'
        title: заголовок графика
        save_path: путь для сохранения (опционально)
        show_info: показывать ли информацию о траекториях
    """
    plt.figure(figsize=(12, 10))
    
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 
              'cyan', 'magenta', 'yellow', 'lime', 'navy', 'maroon']
    
    info_texts = []
    
    for idx, traj_data in enumerate(trajectories_data):
        x_coords = traj_data['x']
        y_coords = traj_data['y']
        label = traj_data.get('label', f'Drone {idx + 1}')
        color = traj_data.get('color', colors[idx % len(colors)])
        
        # Строим траекторию
        plot_trajectory(x_coords, y_coords, label=label, color=color, 
                       show_arrows=(len(trajectories_data) <= 2))  # Стрелки только для 1-2 дронов
        
        # Собираем информацию
        if show_info and len(x_coords) > 1:
            total_distance = calculate_distance(x_coords, y_coords)
            info_texts.append(f"{label}: {len(x_coords)} pts, {total_distance:.2f} m")
    
    plt.xlabel('X coordinate (meters)', fontsize=12)
    plt.ylabel('Y coordinate (meters)', fontsize=12)
    plt.title(title, fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='best')
    plt.axis('equal')
    
    # Добавляем информацию о траекториях
    if show_info and info_texts:
        info_text = '\n'.join(info_texts)
        plt.text(0.02, 0.98, info_text, transform=plt.gca().transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Graph saved to {save_path}")
    
    plt.show()

def main():
    """Основная функция."""
    parser = argparse.ArgumentParser(
        description='Plot trajectory from coordinate log file(s)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Plot trajectory from single log file
  python plot_log_coords.py logs/drone_following_log.txt
  
  # Plot trajectories from two separate files
  python plot_log_coords.py logs/drone_1_log.txt logs/drone_2_log.txt
  
  # Plot with custom title and labels
  python plot_log_coords.py logs/drone_1_log.txt logs/drone_2_log.txt \\
      --title "Leader and Follower" --labels "Leader" "Follower"
  
  # Save plot to file
  python plot_log_coords.py logs/drone_1_log.txt --save trajectory.png
        """
    )
    
    parser.add_argument('log_files', type=str, nargs='+',
                       help='Path(s) to log file(s) with coordinates (1 or more files)')
    parser.add_argument('--title', type=str, default=None,
                       help='Plot title (default: auto-generated)')
    parser.add_argument('--labels', type=str, nargs='+', default=None,
                       help='Labels for each drone trajectory (default: auto-generated)')
    parser.add_argument('--save', type=str, default=None,
                       help='Save plot to file (e.g., trajectory.png)')
    
    args = parser.parse_args()
    
    # Парсим файлы
    trajectories_data = []
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    
    for idx, log_file in enumerate(args.log_files):
        x_coords, y_coords = parse_log_file(log_file)
        
        if x_coords is None or y_coords is None:
            print(f"Warning: Skipping file {log_file} due to parsing errors")
            continue
        
        # Определяем метку
        if args.labels and idx < len(args.labels):
            label = args.labels[idx]
        else:
            # Автоматическая метка на основе имени файла
            base_name = os.path.basename(log_file)
            label = f"Drone {idx + 1} ({base_name})"
        
        trajectories_data.append({
            'x': x_coords,
            'y': y_coords,
            'label': label,
            'color': colors[idx % len(colors)]
        })
    
    if not trajectories_data:
        print("Error: No valid trajectories to plot!")
        return
    
    # Генерируем заголовок
    if args.title is None:
        if len(trajectories_data) == 1:
            args.title = "Drone Trajectory"
        else:
            args.title = f"Multiple Drones Trajectories ({len(trajectories_data)} drones)"
    
    # Строим график
    if len(trajectories_data) == 1:
        # Один дрон - используем старый формат для обратной совместимости
        traj = trajectories_data[0]
        plt.figure(figsize=(12, 10))
        plot_trajectory(traj['x'], traj['y'], title=args.title, 
                       save_path=args.save, label=traj['label'], color=traj['color'])
        plt.xlabel('X coordinate (meters)', fontsize=12)
        plt.ylabel('Y coordinate (meters)', fontsize=12)
        plt.title(args.title, fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        # Информация о траектории
        if len(traj['x']) > 1:
            total_distance = calculate_distance(traj['x'], traj['y'])
            info_text = f"Points: {len(traj['x'])}\nTotal distance: {total_distance:.2f} m"
            plt.text(0.02, 0.98, info_text, transform=plt.gca().transAxes,
                    fontsize=10, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        if args.save:
            plt.savefig(args.save, dpi=300, bbox_inches='tight')
            print(f"Graph saved to {args.save}")
        
        plt.show()
    else:
        # Несколько дронов
        plot_multiple_trajectories(trajectories_data, title=args.title, save_path=args.save)

if __name__ == "__main__":
    main()

