import math
import matplotlib.pyplot as plt

def calculate_triangle_points(pos_start, cup_length=80, cup_count=6, x_direction_sign=1, y_direction_sign=1):
    if x_direction_sign not in {1, -1} or y_direction_sign not in {1, -1}:
        raise ValueError("Direction signs must be 1 (increase) or -1 (decrease).")

    x, y, z, roll, pitch, yaw = pos_start

    points = []

    if cup_count == 1:
        points = [list(pos_start)]  # pos_start를 리스트로 변환하여 추가
    elif cup_count == 3:
        x_offset = x_direction_sign * cup_length * math.cos(math.radians(60))
        y_offset = y_direction_sign * cup_length * math.sin(math.radians(60))
        points.append([round(x - x_direction_sign * cup_length * 1 / 2, 3), round(y - y_direction_sign * cup_length * 1 / 2 * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        points.append([round(x + x_direction_sign * cup_length * 1 / 2, 3), round(y - y_direction_sign * cup_length * 1 / 2 * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        points.append([round(x, 3), round(y + y_direction_sign * (cup_length * 1 / 2 * math.tan(math.radians(60)) - cup_length * 1 / 2 * math.tan(math.radians(30))), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
    elif cup_count == 6:
        # 첫 번째 삼각형의 3개 점
        points.append([round(x - x_direction_sign * cup_length, 3), round(y - y_direction_sign * cup_length * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        points.append([round(x, 3), round(y - y_direction_sign * cup_length * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        points.append([round(x + x_direction_sign * cup_length, 3), round(y - y_direction_sign * cup_length * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])

        # 두 번째 삼각형의 3개 점 (1번 삼각형에 평행한 위치에 배치)
        x_offset = 1 * (((cup_length*1/2)*math.tan(math.radians(30)))**2 + (cup_length*1/2)**2)**(1/2)
        points.append([round(x + x_offset - x_direction_sign * cup_length * 1 / 2, 3), round(y - y_direction_sign * cup_length * 1 / 2 * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        #points.append([round(x + x_offset + x_direction_sign * cup_length * 1 / 2, 3), round(y - y_direction_sign * cup_length * 1 / 2 * math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        points.append([round(x + x_offset, 3), round(y + y_direction_sign * (cup_length * 1 / 2 * math.tan(math.radians(60)) - cup_length * 1 / 2 * math.tan(math.radians(30))), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])


    return points

# 사용 예시
pos_start = [0, 0, 0, 0, 0, 0]  # (x, y, z, roll, pitch, yaw)
cup_length = 80
cup_count = 3  # 여기서 1, 3, 6 중 하나를 선택

# 점 계산
points = calculate_triangle_points(pos_start, cup_length, cup_count)

# 시각화
fig, ax = plt.subplots()

# 점들을 시각화
for point in points:
    ax.plot(point[0], point[1], 'bo')  # x, y 좌표로 점 표시

# 축 설정
ax.set_aspect('equal')
ax.set_xlim(-cup_length * 2, cup_length * 2)
ax.set_ylim(-cup_length * 2, cup_length * 2)
ax.set_title(f"Triangular Pattern with {cup_count} Points")

plt.show()
