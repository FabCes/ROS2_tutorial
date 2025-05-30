# Guida Introduttiva a ROS 2 Humble – Parte 1: Fondamenti Teorici
## Cos'è ROS 2?

ROS 2 (Robot Operating System 2) è un framework open-source pensato per semplificare lo sviluppo di software per robot. Non è un sistema operativo tradizionale, ma una collezione di strumenti, librerie e convenzioni per creare nodi modulari e comunicanti.

ROS 2 si basa su una nuova architettura rispetto a ROS 1 e include:
- Comunicazione distribuita basata su DDS (Data Distribution Service)
- Supporto real-time e multi-threading
- Maggiore sicurezza e portabilità (Linux, Windows, macOS)

---

## Componenti principali

###  Nodo (Node)
Un nodo è un’unità di esecuzione: un processo indipendente che può pubblicare, sottoscrivere, offrire servizi o azioni. Ogni nodo ha un nome univoco nel sistema.

###  Topic
Un topic è un canale sul quale viaggiano i messaggi. I nodi possono:
- **Pubblicare** messaggi su un topic (Publisher)
- **Sottoscriversi** per riceverli (Subscriber)

I topic sono asincroni: i dati fluiscono dal publisher al subscriber senza una connessione diretta.

###  Publisher e Subscriber
- Il **Publisher** crea e invia messaggi a un topic.
- Il **Subscriber** si registra a un topic per ricevere i messaggi pubblicati.

Ogni nodo può essere contemporaneamente publisher e subscriber su più topic.

###  Messaggi (Messages)
I messaggi sono strutture dati predefinite che vengono scambiate sui topic. Alcuni esempi:
- `std_msgs/msg/String`: una semplice stringa
- `geometry_msgs/msg/Twist`: velocità lineare e angolare (per muovere un robot)
- `sensor_msgs/msg/Image`: immagine da una telecamera

---

## Come avviene la comunicazione

Il flusso della comunicazione segue questo schema:

1. Il nodo Publisher pubblica dati su un topic.
2. Il nodo Subscriber ascolta lo stesso topic.
3. ROS 2 collega automaticamente i due nodi tramite DDS.
4. I dati vengono trasferiti senza che i nodi debbano conoscersi.

>  La comunicazione è disaccoppiata: i nodi sono indipendenti e dinamicamente connessi.

---

## Esempio pratico: Talker e Listener

ROS 2 include esempi pronti per comprendere la logica di base.

### 1. Avvia il nodo Publisher:

```bash
ros2 run demo_nodes_py talker
```

**Output:**
```csharp
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
```

### 2. Avvia il nodo Subscriber:
In un secondo terminale:

```bash
ros2 run demo_nodes_py listener
```

**Output:**
```csharp
[INFO] [listener]: I heard: 'Hello World: 1'
[INFO] [listener]: I heard: 'Hello World: 2'
```

---

## Comandi utili da terminale

###  Vedi i topic attivi:

```bash
ros2 topic list
```

**Output:**
```bash
/chatter
/parameter_events
/rosout
```

---

###  Visualizza i messaggi su un topic:

```bash
ros2 topic echo /chatter
```

**Output:**
```yaml
data: Hello World: 3
---
data: Hello World: 4
```

---

### Ottieni informazioni su un topic:

```bash
ros2 topic info /chatter
```

**Output:**
```yaml
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

---

###  Pubblica un messaggio da terminale:

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'ciao da terminale'"
```

---

## Schema logico della comunicazione

```lua
+----------+          /chatter         +-----------+
| talker   |-------------------------> | listener  |
| (pub)    |                          | (sub)     |
+----------+                          +-----------+
```

---

## Riepilogo dei concetti

| Concetto   | Descrizione                                  |
|------------|----------------------------------------------|
| Nodo       | Unità base di esecuzione (processo ROS 2)    |
| Topic      | Canale asincrono per la comunicazione        |
| Publisher  | Nodo che invia dati su un topic              |
| Subscriber | Nodo che riceve dati da un topic             |
| Messaggio  | Struttura dati che viaggia sul topic         |

---

## Domande frequenti

**Q: Posso avere più publisher per un topic?**  
Sì. I subscriber riceveranno dati da tutti i publisher, ma l’ordine non è garantito.

**Q: Posso avere più subscriber per un topic?**  
 Sì. Ogni subscriber riceve una copia del messaggio.

**Q: Cosa succede se avvio il subscriber prima del publisher?**  
 Nessun problema. ROS 2 connette i nodi dinamicamente appena sono disponibili.

---
---
 # Guida Introduttiva a ROS 2 Humble – Parte 2: Comandi e Creazione di un Pacchetto

Questa sezione approfondisce i comandi principali da terminale e spiega passo passo come creare un pacchetto ROS 2 in Python.

---

##  Preparazione dell'ambiente

### 1. Sorgente dell'ambiente ROS 2 Humble

Per rendere disponibili i comandi ROS 2 nel terminale:

```bash
source /opt/ros/humble/setup.bash
```
Puoi anche automatizzarlo nel tuo `.bashrc` così da non doverlo rifare più:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### 2. Creazione di un workspace

L'opzione `-p` nel comando `mkdir -p ~/ros2_ws/src` serve per creare tutte le directory intermedie, se non esistono già, senza generare errori.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

## Creazione di un pacchetto Python

Da dentro la cartella `src` del workspace:

```bash
ros2 pkg create --build-type ament_python nome_pacchetto
```

Questo comando crea una struttura simile a:

```
nome_pacchetto/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── nome_pacchetto
├── nome_pacchetto/
│   └── __init__.py
```

---

##  Cosa modificare nei file

### `setup.py`
Aggiungi gli entry point per i tuoi script:

```python
entry_points={
    'console_scripts': [
        'nome_nodo = nome_pacchetto.nome_file:main',
    ],
},
```

### `package.xml`
Assicurati che siano presenti le dipendenze come:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### Codice del nodo (es. `talker.py` in `nome_pacchetto/`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NomeNodo(Node):
    def __init__(self):
        super().__init__('nome_nodo')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Messaggio {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Pubblicato: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    nodo = NomeNodo()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
```

---

##  Compilazione del pacchetto

Dal workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

###  Perché usare `--symlink-install`?
Crea link simbolici ai file Python invece di copiarli, utile per lo sviluppo: puoi modificare i file senza ricompilare ogni volta.

---

##  Source dell'ambiente

Dopo ogni `colcon build`, ricarica l'ambiente:

```bash
. install/setup.bash
```
---

##  Esecuzione dei nodi

Esegui il nodo con:

```bash
ros2 run nome_pacchetto nome_nodo
```

Assicurati che il nome corrisponda all'`entry_point` definito in `setup.py`.

---

##  Comandi ROS 2 utili

### Elenca i pacchetti disponibili:

```bash
ros2 pkg list
```

### Informazioni su un pacchetto:

```bash
ros2 pkg info nome_pacchetto
```

### Elenca i topic attivi:

```bash
ros2 topic list
```

### Mostra i nodi attivi:

```bash
ros2 node list
```

### Mostra i topic pubblicati da un nodo:

```bash
ros2 node info nome_nodo
```

### Visualizza i messaggi ricevuti:

```bash
ros2 topic echo /nome_topic
```

### Pubblica manualmente un messaggio:

```bash
ros2 topic pub /nome_topic std_msgs/msg/String "data: 'test'"
```

---

##  Differenza tra `ros2 run` e `ros2 launch`

| Comando      | Scopo                                                |
|--------------|------------------------------------------------------|
| `ros2 run`   | Avvia un singolo nodo definito in un pacchetto       |
| `ros2 launch`| Avvia più nodi insieme tramite un file `.launch.py`  |

Esempio di file launch (Python):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nome_pacchetto',
            executable='nome_nodo',
            output='screen'
        ),
    ])
```

Esecuzione:

```bash
ros2 launch nome_pacchetto nome_file.launch.py
```

---

##  Riepilogo

1. Sorgente ROS 2, da non fare se aggiunto con `echo` al `.bash`:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Crea workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   ```
3. Crea pacchetto:
   ```bash
   ros2 pkg create --build-type ament_python nome_pacchetto
   ```
4. Scrivi il nodo in `nome_pacchetto/`
5. Aggiungi entry point in `setup.py`
6. Compila:
   ```bash
   colcon build --symlink-install
   ```
7. Sorgente dell'ambiente:
   ```bash
   source install/setup.bash
   ```
8. Esegui:
   ```bash
   ros2 run nome_pacchetto nome_nodo
   ```

---