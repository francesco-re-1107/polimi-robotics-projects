# Project 1 - Kinematics and Odometry

## How to launch
    ros_launch robotics_project_1 robotics_project_1.launch

## Nodes

### Kinematics
Questo nodo si occupa di calcolare la cinematica del robot a partire dalle singole velocità angolari delle ruote. Le velocità Vx, Vy e ω vengono pubblicate nel topic **/cmd_vel**. Inoltre viene computata la cinematica inversa e le velocità angolari delle ruote vengono pubblicate nel topic **/wheels_rpm**.

### Odometry
Questo nodo si occupa di calcolare l'odometria del robot a partire dalle velocità lineari e angolare del robot utilizzando il metodo di integrazione selezionato tramite configurazione dinamica. La posizione risultante viene pubblicata nel topic **/odom**.

## Topics

### /cmd_vel
Su questo topic vengono pubblicate le velocità Vx, Vy e ω del robot.
Messaggi di tipo **geometry_msgs/TwistStamped**.

### /wheels_rpm
Su questo topic vengono pubblicati i valori di velocità angolare delle ruote calcolati attraverso cinematica inversa.
Messaggi di tipo custom **WheelsRpm**.

### /odom
Su questo topic vengono pubblicati i valori di odometria del robot, quindi x, y e θ.
Messaggi di tipo **nav_msgs/Odometry**.

## Services

### /reset
Questo servizio si occupa di resettare la posizione del robot per il nodo Odometry.
    
    float64 x
    float64 y
    float64 theta
    ---


### /reset_kinematics
Questo servizio si occupa di resettare la cinematica del robot per il nodo Kinematics.
    
    float64 x
    float64 y
    float64 theta
    ---

## Static parameters

### Posizione iniziale
Questi parametri indicano la posizione iniziale del robot rispetto all'origine.
- initial_pose/x
- initial_pose/y
- initial_pose/theta

## Dynamic parameters

### /integration_method
Questo parametro si occupa di impostare il metodo di integrazione che può essere:
1. Euler (EULER)
2. Runge-Kutta (RUNGE_KUTTA)

### /wheels_radius
Questo parametro si occupa di settare il raggio della ruota.

### /cpr
Questo parametro si occupa di settare il numero di ticks per rotazione.

### /width
Questo parametro si occupa di settare la posizione della ruota rispetto alla lunghezza del robot.

### /length
Questo parametro si occupa di settare la posizione della ruota rispetto alla larghezza del robot.

## Custom messages

### WheelsRpm
Questo messaggio contiene le velocità angolari delle ruote calcolate attraverso la cinematica inversa. Valori in rad/min.

    Header header
    float64 rpm_fl
    float64 rpm_fr
    float64 rpm_rr
    float64 rpm_rl

## TF

### world -> odom
Viene ottenuta con una trasformazione **statica** da world a odom definita nel launch file.

### odom -> base_link
Viene ottenuta con una trasformazione **dinamica** pubblicata dal nodo **Odometry**.

## Calibration

La calibrazione è stata fatta in maniera automatica attraverso uno script python (script/evaluator*.py). In tutti i test è stata utilizzata la bag3 e Runge-Kutta come metodo di integrazione.

Per ogni combinazione dei parametri di calibrazione lo script si occupa di:
1. Impostare i parametri di test
2. Resettare la posizione del robot
3. Eseguire la bag
4. Calcolare il Mean Square Error relativo a:
    - Distanza tra posizione calcolata e posizione reale del robot.
    - Differenza tra theta calcolato e theta reale del robot.
    - I due valori precedenti vengono poi mediati assieme per ottenere un valore di errore unico.
5. Salvare i dati in un file csv
6. Produrra una heatmap a partire dal file csv. (vedi allegato.png)

I parametri di configurazione sono 4 ma, attraverso manipolazione delle formule della cinematica, possono essere ricondotti a 2 soltanto.
- P1 = WR/CPR
- P2 = W (oppure L , essendo questi due valori sommati assieme)

(Dove WR = raggio ruota, CPR = Counts Per Revolution, L = parametro length, W = parametro width) 

I risultati migliori sono stati i seguenti (vedi allegato.png):
- P1 = 0.001813 => WR = 0.076146 e CPR = 42.0
- L = 0.2
- W = 0.169