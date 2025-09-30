from controller import Robot
from maze_nodes import Node, Dfso
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random


# Matriz do labirinto (definições iniciais) (Variaveis globais)
MAZE = np.array(np.zeros((15, 15)))
position = [7 ,7 ,'E']
_maze_fig = None
_maze_ax = None

def set_speed(left, right):
    # left e right são as velocidades lineares das rodas em (m/s)
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

def turn(direction='left', times=1, speed=1, stop=True):
    """Gira o robô para a esquerda ou direita.
    direction: 'left' ou 'right'"""

    if direction == 'left':
        set_speed(-speed, speed)  # Motor esquerdo para trás, motor direito para frente
    elif direction == 'right':
        set_speed(speed, -speed)  # Motor esquerdo para frente, motor direito para trás
    elif direction == 'front':
        set_speed(speed, speed)
    elif direction == 'back':
        set_speed(-speed, -speed)
    else:
        raise ValueError("Direção inválida. Use 'left' ou 'right'.")
    
    for _ in range(times):
        robot.step(timestep)
    
    if stop:
        set_speed(0, 0)  # Para os motores após o giro

'''def rotate(angle):

    angular_speed = 0.5908 # velocidade angular do robo (rad/s)
    angle_rad = np.radians(angle)  # Converte o ângulo para radianos

    # Tempo necessário para girar o ângulo desejado
    rotation_time = abs(angle_rad) / angular_speed

    # Configura a direção do giro
    if angle > 0:  # Sentido anti-horário
        set_speed(-1, 1)
    else:  # Sentido horário
        set_speed(1, -1)

    # Executa o giro pelo tempo necessário
    start_time = robot.getTime()
    while robot.getTime() - start_time < rotation_time:
        robot.step(timestep)

    # Para os motores após o giro
    set_speed(0, 0)'''

#TODO: move_on_edge com controlade PID (incompleto: não adaptar)
def rotate(angle, linear_speed=1):

    # 0.5908 is the robot's angular speed in rad/s, used to calculate the turning radius (r)
    r = 1 / 0.5908  # raio de giro do robo (m)
    speed = linear_speed
    angular_speed = linear_speed / r # velocidade angular do robo (rad/s)

    if angle > 0:
        angle_rad = abs(np.radians(angle+1.152))  # Converte o ângulo para radianos
    else:
        angle_rad = abs(np.radians(angle-1.152))  # Converte o ângulo para radianos
    
    dt = timestep / 1000  # Converte o timestep para segundos        
    DA = 0
    errorIntegral = 0
    start_time = robot.getTime()    
    Kp = 3
    Kd = 1
    Ki = 0.0001

    while True: # gira até completar o ângulo desejado    

        # Configura a direção do giro
        if angle > 0:  # Sentido anti-horário
            set_speed(-speed, speed)
        else:  # Sentido horário
            set_speed(speed, -speed)
        robot.step(timestep)

        angular_speed = speed / r # velocidade angular do robo (rad/s)
        t = robot.getTime() - start_time # Tempo decorrido
        
        DA = DA + angular_speed * dt  # Ângulo girado até agora
        dA = angular_speed * dt  # Incremento do ângulo girado
        angleError = angle_rad - DA
        errorIntegral = errorIntegral + angleError*dt
        speed = (Kp * angleError + Ki * errorIntegral + Kd * dA)/angle_rad        

        print(f'Remaining:{np.degrees(angleError):.4f}°,Rotated:{DA:.4f} | Speed:{speed:.2f} m/s,Time: {t:.2f}s')

        if abs(angleError) <= 0.00001: # Finaliza o giro ao completar o ângulo ou após 10 segundos
            break

    # Para os motores após o giro
    set_speed(0, 0)

def probe_direction(times=0, decimals=1):
    """
    Realiza leituras dos sensores de distância e retorna a média das leituras.
    times: número de leituras a serem feitas (0 para uma única leitura)
    """
    n_sensors = len(sensores)

    if times <= 0:
        robot.step(timestep)
        return [sensor.getValue() for sensor in sensores]

    readings = np.empty((times, n_sensors), dtype=float)

    for i in range(times):
        robot.step(timestep)
        readings[i] = [sensor.getValue() for sensor in sensores]

    return np.round(readings.mean(axis=0), decimals).tolist()

def probe_for_walls(): 
    """
    Analisa as paredes ao redor do robô e retorna uma lista indicando a presença de paredes
    em relação às direções absolutas do labirinto [N, E, S, W].
    """
    directions = ['N', 'E', 'S', 'W']  # Ordem fixa das direções absolutas
    wall_detection = [0, 0, 0, 0]  # Inicializar detecção de paredes
    probe_detection = [0, 0, 0, 0]
    min_return = 920

    # Identificar o índice da direção atual no labirinto
    current_dir_index = directions.index(position[2])

    
    # Faz duas leituras médias para frente e trás
    probe = probe_direction(times=3)
    probe_detection[0], probe_detection[1] = probe[0], probe[7]

    rotate(90)
    robot.step(timestep)

    probe = probe_direction(times=3)
    probe_detection[2], probe_detection[3] = probe[0], probe[7]
    
    rotate(-90)
    robot.step(timestep)

    if current_dir_index == 0: # Direcionamento Norte

        if probe_detection[0] >= min_return:
            wall_detection[3] = 1 #Oeste
        if probe_detection[1] >= min_return:
            wall_detection[1] = 1 #Leste
        
        if probe_detection[2] >= min_return:
            wall_detection[2] = 1 #Sul
        if probe_detection[3] >= min_return:
            wall_detection[0] = 1 #Norte

    elif current_dir_index == 1: # Direcionamento Leste
        
        if probe_detection[0] >= min_return:
            wall_detection[0] = 1 #Norte
        if probe_detection[1] >= min_return:
            wall_detection[2] = 1 #Sul
        
        if probe_detection[2] >= min_return:
            wall_detection[3] = 1 #Oeste
        if probe_detection[3] >= min_return:
            wall_detection[1] = 1 #Leste

    elif current_dir_index == 2: # Direcionamento Sul
        
        if probe_detection[0] >= min_return:
            wall_detection[1] = 1 #Leste
        if probe_detection[1] >= min_return:   
            wall_detection[3] = 1 #Oeste

        if probe_detection[2] >= min_return:
            wall_detection[0] = 1 #Norte
        if probe_detection[3] >= min_return:
            wall_detection[2] = 1 #Sul

    elif current_dir_index == 3: # Direcionamento Oeste
        
        if probe_detection[0] >= min_return:
            wall_detection[2] = 1 #Sul
        if probe_detection[1] >= min_return:
            wall_detection[0] = 1 #Norte

        if probe_detection[2] >= min_return:
            wall_detection[1] = 1 #Leste
        if probe_detection[3] >= min_return:
            wall_detection[3] = 1 #Oeste     
    
    
    return wall_detection

def update_maze(position=position, maze=MAZE):
    """
    Atualiza a matriz do labirinto com as informações de parede obtidas.
    """
    # Atualiza a matriz do labirinto com as informações de parede
    directions = ['N', 'E', 'S', 'W']
    walls = probe_for_walls()
    for i in range(4):
        if walls[i] == 1:
            if directions[i] == 'N':
                maze[position[0] - 1][position[1]] = 1
            if directions[i] == 'E':
                maze[position[0]][position[1] + 1] = 1
            if directions[i] == 'S':
                maze[position[0] + 1][position[1]] = 1
            if directions[i] == 'W':
                maze[position[0]][position[1] - 1] = 1

    global MAZE
    MAZE = maze
    print(MAZE)

    return walls

def show_mazegraph(G, dfso):
    global _maze_fig, _maze_ax

    node_colors = []
    for node in G.nodes():
        if "customNode" in G.nodes[node]:
            if G.nodes[node]["customNode"].name == dfso.actualState.name:
                node_colors.append('#c72626')
            elif G.nodes[node]["customNode"].name in dfso.visitedStates:
                node_colors.append('orange')
            elif G.nodes[node]["customNode"] in dfso.visibleStates:
                node_colors.append('#4883db')
            else:
                node_colors.append('green')
        else:
            node_colors.append('gray')  # Cor padrão para outros nós

    pos = {(x, y): (y, -x) for x, y in G.nodes()}

    if _maze_fig is None or _maze_ax is None:
        _maze_fig, _maze_ax = plt.subplots(figsize=(6, 6))
        plt.ion()
        plt.show()

    _maze_ax.clear()
    nx.draw(G, pos, ax=_maze_ax, with_labels=True, node_color=node_colors, node_size=200, edge_color='gray', font_size=6)
    _maze_ax.set_title("Maze Graph")
    _maze_ax.axis("equal")
    _maze_fig.canvas.draw()
    _maze_fig.canvas.flush_events()       

#TODO: move_on_edge com controlade PID (incompleto: não adaptar)
def move_on_edge(direction, step_distance=10.91581, linear_speed=1.0, freeMove=False):
    #10.3467
    """
    Move o robô 2 metros na direção especificada (N, S, E ou W).
    No final da execução o robo apontara para a direção de movimento
    """
    
    dir_clockwise = ['N', 'E', 'S', 'W']
    target_dir_index = dir_clockwise.index(direction)
    actual_dir_index = dir_clockwise.index(position[2])
    roll = (target_dir_index - actual_dir_index) % 4

    # aleatoriza o sentido de rotação (horário ou anti-horário)
    clockwise = random.choice([True, False])
    if roll != 0:
        if clockwise:
            angle = -90 * roll
        else:
            angle = 90 * (4 - roll)
        rotate(angle)       
    
    if not freeMove:
        position[2] = direction
        #atualiza a posição do robô
        if direction == 'N':
            position[0] -= 2
        elif direction == 'E':
            position[1] += 2
        elif direction == 'S':
            position[0] += 2
        elif direction == 'W':
            position[1] -= 2    


    # Movimentar para frente
    speed = linear_speed
    distanceError = 0
    integralError = 0
    start_time = robot.getTime()
    dt = timestep / 1000
    DL = 0
    Kp = 12
    Kd = 5
    Ki = 0.0001

    while True:
        set_speed(speed, speed) 
        robot.step(timestep)
        t = robot.getTime() - start_time

        DL = DL + speed * dt
        dL = speed * dt
        distanceError = step_distance - DL
        integralError = integralError + distanceError*dt
        speed = (Kp * distanceError + Ki * integralError + Kd * dL)/step_distance

        #print(f'Remaining:{np.degrees(distanceError):.4f}, Walked:{DL:.4f} | Speed:{speed:.2f} m/s,Time: {t:.2f}s')

        if abs(distanceError) <= 0.00001:
            break
    # Parar o robô
    set_speed(0, 0)

    # Ajuste fino para alinhar o robô
    turn('front', 2, 0.343) 

def move_to_node(no_atual, destino):
    '''Move o robo para o nó destino especificado.
      O nó atual é o caminho que o robo usou para chegar ao nó atual.
      O nó destino é o caminho que o usará para visitar o seu destino.
    '''
    caminho_comum = ''
    for c in range(len(no_atual)):
        try:
            if no_atual[c] == destino[c]:
                caminho_comum = caminho_comum + destino[c]
            else:
                break
        except IndexError:
            break
    
    if caminho_comum != '':
        if len(no_atual) > len(destino):
            caminho_para_destino = no_atual[len(caminho_comum):][::-1]  # Caminho de volta ao ponto comum
            for i in caminho_para_destino:
                if i == 'N':
                    move_on_edge('S')
                elif i == 'E':
                    move_on_edge('W')
                elif i == 'S':
                    move_on_edge('N')
                elif i == 'W':
                    move_on_edge('E')
                else:
                    print('Erro: direção inválida')
                    break
        else:
            caminho_para_destino = destino[len(caminho_comum):]
            for i in caminho_para_destino:
                move_on_edge(i)
            
    else:
        caminho_para_centro = no_atual[::-1]
        for i in caminho_para_centro:
        
            if i == 'N':
                move_on_edge('S')
            elif i == 'E':
                move_on_edge('W')
            elif i == 'S':
                move_on_edge('N')
            elif i == 'W':
                move_on_edge('E')
            else:
                print('Erro: direção inválida')
                break 
        
        destino_final = destino
        for i in destino_final:
            move_on_edge(i)            

def dijkstra_way(G, start, end):
    G = nx.Graph(G)  # Cria uma cópia do grafo para evitar modificações indesejadas

    no_i = (start.x, start.y)  # Obtém o nó inicial
    no_f = (end.x, end.y)  # Obtém o nó final

    path = nx.dijkstra_path(G, source= no_i, target= no_f)  # usa BFS internamente    
    way = []

    for node in path:
        way.append(G.nodes[node]["customNode"].path)
    
    print(way)    
    n = way[0] # Começa no primeiro caminho

    for c, path in enumerate(way):
        move_to_node(n, path)
        n = path

def ajust_position(ciclos=1, metodo=1):
    n = 0
    n_ciclos = ciclos*672
    clockwise = random.choice([True, False])
    # Para paredes em L ou U
    if metodo == 1:
        while n < n_ciclos: # numero de iterações para uma volta completa

            if n % 672 == 0:
                if clockwise == True:
                    clockwise = False
                else:
                    clockwise = True
            
            if clockwise:
                turn('right', 1, 0.5)
            else:
                turn('left', 1, 0.5)
            
            probe = probe_direction(times=5, decimals=2)
            
            if (probe[3] > 954) and (probe[4] > 954): #Numeros experimentais
                turn('back', 1, 1)
            elif (850 < probe[3] < 954) and (850 < probe[4] < 954):
                turn('front', 1, 1)

            n += 1
    # Para paredes em H
    elif metodo == 2:
        while n < n_ciclos: # numero de iterações para uma volta completa

            if n % 672 == 0:
                if clockwise == True:
                    clockwise = False
                else:
                    clockwise = True
            
            if clockwise:
                turn('right', 1, 0.5)
            else:
                turn('left', 1, 0.5)
            
            probe = probe_direction(times=5, decimmals=2)

            if (probe[3] > 954) and (probe[4] > 954): #Numeros experimentais
                turn('back', 1, 1)
            elif (900 < probe[3] < 954) and (900 < probe[4] < 954):
                turn('front', 1, 1)

            n += 1
    elif metodo == 3:
    
        max_value = 0
        actual_value = 0
        
        # gira 360 graus procurando o valor máximo
        while n < 672:
            if clockwise:
                turn('right', 1, 0.5)
            else:
                turn('left', 1, 0.5)
            
            probe = probe_direction(times=30, decimmals=4)
            if probe[0] > max_value:                
                max_value = probe[0]            
            n += 1
        print(f'Valor máximo encontrado: {max_value}')

        # inverte o sentido de rotação
        if clockwise == True:
            clockwise = False
        else:
            clockwise = True
            
        # gira até encontrar o valor máximo
        while True:
            probe = probe_direction(times=30, decimmals=4)
            actual_value = probe[0]
            if abs(actual_value - max_value) > 1:
                if clockwise:
                    turn('right', 1, 0.5)
                else:
                    turn('left', 1, 0.5)
            else:
                print('Alinhado')
                print(f'Valor atual: {actual_value}')
                break

def walls_in_shapeOf(walls, shape='L'):
    l_shaped_patterns = {(0, 0, 1, 1), (0, 1, 1, 0), (1, 1, 0, 0), (1, 0, 0, 1)}
    U_shaped_patterns = {(0, 1, 1, 1), (1, 1, 1, 0), (1, 1, 0, 1), (1, 0, 1, 1)}
    H_shaped_patterns = {(1, 0, 1, 0), (0, 1, 0, 1)}
    if shape == 'L':
        if tuple(walls) in l_shaped_patterns:
            return True
    elif shape == 'U':
        if tuple(walls) in U_shaped_patterns:
            return True
    elif shape == 'H':
        if tuple(walls) in H_shaped_patterns:
            return True

def mapping_maze(MAZE=MAZE, position=position):
    '''mapeia o labirinto e retorna um grafo com as informações do labirinto'''

    # Inicializa o grafo
    maze_dimensions = MAZE.shape
    G = nx.Graph()
    for x in range(maze_dimensions[0]):
        for y in range(maze_dimensions[1]):
            G.add_node((x, y))

    #definir o no inicial ou no raiz
    root_node = Node(position=position, path='', walls=update_maze())
    G.nodes[(root_node.x, root_node.y)]["customNode"] = root_node

    # Cria o objeto de interação com o DFS
    dfso = Dfso()
    dfso.actualState = root_node
    dfso.visitedStates.append(root_node.name)    

    #direções absolutas
    directions = ['N', 'E', 'S', 'W']

    #contagem de ciclos
    n = 0
        
    MazeFinished = False

    #<--- CICLO DE MAPEAMENTO --->
    while not MazeFinished:
        print('<======[Ciclo de mapeamento]======>')

        #<--- CICLO DE EXPLORAÇÂO --->
        # Checar se exitem caminhos explorados
        if n == 0 :
            new_ways = False
            travel = False
        else:
            for neighbor in dfso.actualState.neighbours:
                # if True (Há algum caminho explorado):
                if neighbor.name not in dfso.visitedStates: #Estados visitados é uma lista de Strings
                    # Ir para o ciclo de visitação                
                    new_ways = True
                # else (não há caminhos explorados):
                else:
                    new_ways = False

        if not new_ways:             
            # mapear novos caminhos
            walls = update_maze(position=position.copy(), maze=MAZE)
            print(walls, 'Aqui')
            moves = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # N, E, S, W
            for i, (dx, dy) in enumerate(moves):
                if walls[i] == 0:
                    
                    next_position = [dfso.actualState.x + dx, dfso.actualState.y + dy]
                    
                    try:
                        if G.nodes[(next_position[0], next_position[1])]["customNode"].name not in dfso.visitedStates:
                            new_node = Node(position=next_position, path=dfso.actualState.path + directions[i])
                            
                            #atualizar dfso
                            dfso.visibleStates.appendleft(new_node)
                            dfso.set_mappedStates()
                            dfso.actualState.neighbours.add(new_node)
                            dfso.actualState.walls = walls
                            new_node.neighbours.add(dfso.actualState)

                            G.nodes[(new_node.x, new_node.y)]["customNode"] = new_node
                            G.add_edge((dfso.actualState.x, dfso.actualState.y), (new_node.x, new_node.y))

                        new_ways = True
                    except KeyError:
                        new_node = Node(position=next_position, path=dfso.actualState.path + directions[i])
                            
                        #atualizar dfso
                        dfso.visibleStates.appendleft(new_node)
                        dfso.set_mappedStates()
                        dfso.actualState.neighbours.add(new_node)
                        dfso.actualState.walls = walls
                        new_node.neighbours.add(dfso.actualState)

                        G.nodes[(new_node.x, new_node.y)]["customNode"] = new_node
                        G.add_edge((dfso.actualState.x, dfso.actualState.y), (new_node.x, new_node.y))

                        new_ways = True
                    
                else:
                    print(f"parede ao {directions[i]}")
            show_mazegraph(G, dfso)
            
        
            # if True (Caminhos novos mapeados com sucesso):
            if new_ways:
                # atualizar posição e ir para o ciclo de visitação
                dfso.actualState.walls = walls

            # else (não há caminhos novos):
            else:
                print('não há caminhos novos')                
                # buscar estados guardados
                for state in dfso.visibleStates:
                    # if True (há estados guardados):
                    if state not in dfso.actualState.neighbours:                       
                        # vai para o ciclo de viagem
                        travel= True
            
                    # else (não há estados guardados):
                    else:            
                        # parar o robô e finalizar o programa
                        MazeFinished = True
        
        #<--- CICLO DE AJUSTE --->
        '''ajust_condition = (walls_in_shapeOf(dfso.actualState.walls, 'L') or walls_in_shapeOf(dfso.actualState.walls, 'U')) and n > 2 and n % 2 == 0
        if ajust_condition:
            ajust_position(ciclos=2, metodo=1)'''
        
        
        if (not MazeFinished) and (not travel):
            #<--- CICLO DE VISITAÇÃO --->
            # Escolhe um nó entre os explorados para poder visitar e os nós que sombram são guardados
            print('<======[Ciclo de visitação]======>')
            target = dfso.visibleStates[0]
            if target in dfso.actualState.neighbours:
                print(f'visitando {target.name}')
                move_on_edge(target.path[-1])
                dfso.actualState = target
                dfso.visitedStates.append(dfso.actualState.name)
                dfso.visibleStates.popleft()
                dfso.set_mappedStates()
            else:
                travel = True            
            show_mazegraph(G, dfso)

        
        if (not MazeFinished) and travel:
            #<--- CICLO DE VIAGEM --->            
            # Executa uma "viagem" ao estado guardado e o visita
            print('<======[Ciclo de viagem]======>')
            target = dfso.visibleStates[0]
            print(f'viajando para {target.name}')
            dijkstra_way(G, dfso.actualState, target)
            dfso.actualState = target
            dfso.visitedStates.append(dfso.actualState.name)
            dfso.visibleStates.remove(target)
            dfso.set_mappedStates()
            travel = False
            show_mazegraph(G, dfso)
            

        n = n + 1  
        print(f'{n}')
    
# Configurações iniciais do robô 
robot = Robot()
timestep = int(robot.getBasicTimeStep())


# Inicializar motores
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# inicializa sensores
left_pos_sensor = robot.getDevice('left wheel sensor')   
right_pos_sensor = robot.getDevice('right wheel sensor')
so0 = robot.getDevice('so0')
so1 = robot.getDevice('so1')
so2 = robot.getDevice('so2')
so3 = robot.getDevice('so3')
so4 = robot.getDevice('so4')
so5 = robot.getDevice('so5')
so6 = robot.getDevice('so6')
so7 = robot.getDevice('so7')
left_pos_sensor.enable(timestep)
right_pos_sensor.enable(timestep)
sensores = [so0, so1, so2, so3, so4, so5, so6, so7]
for sensor in sensores:
    sensor.enable(timestep)

