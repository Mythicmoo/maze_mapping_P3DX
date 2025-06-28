from controller import Robot
from maze_nodes import Node, Dfso
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random


# Matriz do labirinto (definições iniciais)
MAZE = np.array(np.zeros((15, 15)))
position = [7 ,7 ,'E']

def set_speed(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

def turn(direction='left', times=1, speed=1):
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
    
    set_speed(0, 0)  # Para os motores após o giro
    
def rotate(angle):

    angular_speed = 0.5908 
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
    set_speed(0, 0)

def probe_direction():
    """Lê as medidas dos sensores."""
    robot.step(timestep)
    distances = []
    for sensor in sensores:
        distance = float('{:.1f}'.format(sensor.getValue()))
        distances.append(distance)
    return distances

#precisa de melhorias
def probe_for_walls():
    """
    Analisa as paredes ao redor do robô e retorna uma lista indicando a presença de paredes
    em relação às direções absolutas do labirinto [N, E, S, W].
    """
    directions = ['N', 'E', 'S', 'W']  # Ordem fixa das direções absolutas
    wall_detection = [0, 0, 0, 0]  # Inicializar detecção de paredes

    # Identificar o índice da direção atual no labirinto
    current_dir_index = directions.index(position[2])

    for _ in range(4):
        # Capturar leituras dos sensores
        probe = probe_direction()

        # Determinar se há uma parede à frente (critério ajustável conforme o sensor)
        if probe[3] > 930 or probe[4] > 930:
            wall_detection[current_dir_index] = 1  # Parede detectada

        # Atualizar a direção atual
        current_dir_index = (current_dir_index + 1) % 4
        
        # Rotacionar o robô 90 graus para a próxima direção
        rotate(-90)
    
    # Retornar à orientação original
    rotate(-90 * (current_dir_index - directions.index(position[2])) + 2.3)  #2 é o erro
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
    plt.figure(figsize=(4,4))
    nx.draw(G, pos, with_labels=True, node_color=node_colors, node_size=200, edge_color='gray', font_size=6)
    plt.title("Grafo de Grid 2D com conexão ao nó do Norte")
    plt.axis("equal")
    plt.show()

def move_along_edge(traslation_time=14.17, speed=1.5):

    startTime = robot.getTime()
    while robot.getTime() - startTime < traslation_time:

        d = 0
        for _ in range(2):
            probe = probe_direction() 
            d = (d + probe[0] - probe[7])/2
        
        print(robot.getTime())
        if -14 < d < 14:
            turn('front', 2, speed)  # Se não houver paredes, continua em frente
        elif (14 <= d <= 50):
            turn('left', 1, speed)
        elif (-50 >= d >= -14):
            turn('right', 1, speed)        
        else:
            turn('front', 2, speed)  # Se não houver paredes, continua em frente
            pass       

def move_on_edge(direction):
    """
    Move o robô 2 metros na direção especificada (N, S, E ou W).
    No final da execução o robo apontara para a direção de movimento
    """
    directions = ['N', 'E', 'S', 'W']
    current_dir_index = directions.index(direction) # direção de movimento desejada
    translation_time = 10.26227291 # Tempo necessário para percorrer 2 metros (distanciaDeUmPasso_emmetros/velocidade_linear)
   

    rotate(-90 * (current_dir_index - directions.index(position[2])))
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
    start_time = robot.getTime()
    set_speed(1, 1)  # Velocidade constante
    while robot.getTime() - start_time < translation_time:
        robot.step(timestep)

    # Parar o robô
    set_speed(0, 0)

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

def Dijkstra_way(G, start, end):
    G = nx.Graph(G)  # Cria uma cópia do grafo para evitar modificações indesejadas

    no_i = (start.x, start.y)  # Obtém o nó inicial
    no_f = (end.x, end.y)  # Obtém o nó final

    path = nx.dijkstra_path(G, source= no_i, target= no_f)  # usa BFS internamente    
    print(path)
    way = []
    for node in path:
        way.append(G.nodes[node]["customNode"].path)
    print(way)
    
    n = way[0] # Começa no primeiro caminho    
    for path in way:
        move_to_node(n, path)
        n = path
    
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
        print('Ciclo de mapeamento')

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
            moves = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # N, E, S, W
            for i, (dx, dy) in enumerate(moves):
                if walls[i] == 0:
                    
                    next_position = [dfso.actualState.x + dx, dfso.actualState.y + dy]
                    
                    try:
                        if G.nodes[(next_position[0], next_position[1])]["customNode"].name not in dfso.visitedStates:
                            new_node = Node(position=next_position, path=dfso.actualState.path + directions[i], walls=walls)
                            
                            #atualizar dfso
                            dfso.visibleStates.appendleft(new_node)
                            dfso.set_mappedStates()
                            dfso.actualState.neighbours.add(new_node)
                            new_node.neighbours.add(dfso.actualState)

                            G.nodes[(new_node.x, new_node.y)]["customNode"] = new_node
                            G.add_edge((dfso.actualState.x, dfso.actualState.y), (new_node.x, new_node.y))

                        new_ways = True
                    except KeyError:

                        new_node = Node(position=next_position, path=dfso.actualState.path + directions[i], walls=walls)
                            
                        #atualizar dfso
                        dfso.visibleStates.appendleft(new_node)
                        dfso.set_mappedStates()
                        dfso.actualState.neighbours.add(new_node)
                        new_node.neighbours.add(dfso.actualState)

                        G.nodes[(new_node.x, new_node.y)]["customNode"] = new_node
                        G.add_edge((dfso.actualState.x, dfso.actualState.y), (new_node.x, new_node.y))

                        new_ways = True
                    
                else:
                    print('caminho bloquedo')
            show_mazegraph(G, dfso)
            
        
            # if True (Caminhos novos mapeados com sucesso):
            if new_ways:
                # ir para o ciclo de visitação
                pass
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
         
        
        if (not MazeFinished) and (not travel):
            #<--- CICLO DE VISITAÇÃO --->
            # Escolhe um nó entre os explorados para poder visitar e os nós que sombram são guardados
            print('Ciclo de visitação')
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
            print('Ciclo de viagem')
            target = random.choice(dfso.visibleStates)
            Dijkstra_way(G, dfso.actualState, target)
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

# Inicializar motores e sensores
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)
so0 = robot.getDevice('so0')
so1 = robot.getDevice('so1')
so2 = robot.getDevice('so2')
so3 = robot.getDevice('so3')
so4 = robot.getDevice('so4')
so5 = robot.getDevice('so5')
so6 = robot.getDevice('so6')
so7 = robot.getDevice('so7')
sensores = [so0, so1, so2, so3, so4, so5, so6, so7]
for sensor in sensores:
    sensor.enable(timestep)


