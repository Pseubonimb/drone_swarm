# Пытался добавить ветер в симуляцию

from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Настройка ветра (в ньютонах)
#WIND_FORCE = [105.2, 0.5, 0.0]  # X, Y, Z
WIND_VELOCITY = [5.0, 2.0, 0.0]  # м/с в ENU: 5 м/с на восток, 2 м/с на север

# Получаем корень сцены и ищем все Iris-дроны
root = supervisor.getRoot()
children_field = root.getField("children")

drones = []
for i in range(children_field.getCount()):
    node = children_field.getMFNode(i)
    if node.getTypeName() == "Iris":
        drones.append(node)

print(f"Wind supervisor: найдено {len(drones)} дронов")

# Основной цикл
while supervisor.step(timestep) != -1:
    for drone in drones:
        wind_field = drone.getField("wind_force")
        if wind_field:
            wind_field.setSFVec3f(WIND_VELOCITY)
        else:
            print("Ошибка: у дрона нет поля wind_force!")