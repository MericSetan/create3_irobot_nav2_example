#!/usr/bin/env python3

maze = [
    [0, 0, 0, 0],
    [1, 1, 0, 1],
    [0, 0, 0, 0],
    [0, 1, 1, 0]
]
maze_w_coordinates = [
    [(12.5, 0, 2), (5.7, 6.9), (9.87, 1.23), (7.0, 6.58)],
    [(3.14, 2.71), (8.77, 8.56), (8.8, 4.0), (6.79, 9.12)],
    [(5.98, 48), (9.0, 10.0), (1.23, 4.56), (11.11, 12.12)],
    [(15.0, 16.0), (7.77, 8.88), (17.17, 18.18), (0.11, 0.22)]
]

gezilen_hucre = []
def is_valid_move(x, y):
    if x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0]):
        return False
    return maze[x][y] == 0

def dfs(x, y):
    if not is_valid_move(x, y):
        return
    
    maze[x][y] = -1  # Ziyaret edildi olarak işaretle
    #print(f"Ziyaret edilen hücre: ({x}, {y})")
    gezilen_hucre.append((x,y))
    
    dx = [0, 0, -1, 1]
    dy = [-1, 1, 0, 0]
    for i in range(4):
        new_x = x + dx[i]
        new_y = y + dy[i]
        dfs(new_x, new_y)

dfs(0, 0)
# dizi boyutları eşit mi kontrolü yaabilirsin
print(maze)

selected_values = []

for index in gezilen_hucre:
    row, col = index
    value = maze_w_coordinates[row][col]
    selected_values.append(value)
print(selected_values)
if selected_values ==[]:
    print("initial pose yanlis secildi")
