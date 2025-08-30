"""
Controlador PID para equilibrar un péndulo invertido en Webots R2025a.
Incluye perturbaciones manuales con las teclas A y D (versión corregida y mejorada).

Cambios clave:
- El PID se ejecuta en cada paso, no solo cuando no se presionan teclas.
- La fuerza de perturbación se SUMA a la fuerza del PID, en lugar de reemplazarla.
- Se ha corregido el signo de la fuerza aplicada para una reacción correcta.
- Se ha añadido un control secundario para mantener el carro cerca del centro.
- Se ha añadido un límite a la velocidad del motor para evitar fuerzas irreales.
"""

from controller import Robot, Keyboard

# --- Parámetros de Simulación y PID ---
TIME_STEP = 16
MAX_SPEED = 100.0  # Límite de velocidad para el motor

# --- Constantes del PID (VALORES RE-SINTONIZADOS) ---
# Kp reacciona al error actual (ángulo y posición).
Kp = 200  #150
# Ki corrige errores residuales a largo plazo. Se mantiene bajo.
Ki = 0.05  #0.1
# Kd amortigua la reacción y previene oscilaciones.
Kd = 0.5  #25.0

# --- Pesos para el error combinado ---
# La prioridad es mantener el ángulo (mayor peso).
ANGLE_WEIGHT = 0.8
# La segunda prioridad es mantener el carro centrado.
POSITION_WEIGHT = 0.2

# --- Parámetros de Perturbación ---
PERTURBATION_FORCE = 25 # Aumentamos un poco para que se note el empuje

# Inicialización de variables del PID
integral = 0.0
previous_error = 0.0

# --- Creación y Configuración del Robot ---
robot = Robot()

# Obtener dispositivos
pole_angle_sensor = robot.getDevice("pole position sensor")
cart_position_sensor = robot.getDevice("cart position sensor")
cart_motor = robot.getDevice("cart motor")

# Habilitar el teclado
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Habilitar los sensores
pole_angle_sensor.enable(TIME_STEP)
cart_position_sensor.enable(TIME_STEP)

# Configurar el motor para control de fuerza/velocidad
cart_motor.setPosition(float('inf'))
cart_motor.setVelocity(0.0)

print("Controlador PID corregido iniciado para Webots R2025a.")
print("Presiona la ventana de simulación y usa 'A' y 'D' para empujar el carro.")
print(f"Sintonización actual: Kp={Kp}, Ki={Ki}, Kd={Kd}")

# --- Bucle de Control Principal ---
while robot.step(TIME_STEP) != -1:
    # --- 1. Lectura de Sensores ---
    pole_angle = pole_angle_sensor.getValue()
    cart_position = cart_position_sensor.getValue()

    # --- 2. Cálculo del PID (se ejecuta siempre) ---
    # El error es una combinación ponderada del ángulo y la posición del carro.
    # El objetivo es que tanto el ángulo como la posición sean 0.
    error = (ANGLE_WEIGHT * pole_angle) + (POSITION_WEIGHT * cart_position)

    # El término integral solo se acumula si el error es significativo,
    # para evitar "windup" cuando el sistema está casi estable.
    if abs(error) > 0.01:
        integral += error * (TIME_STEP / 1000.0)
    
    derivative = (error - previous_error) / (TIME_STEP / 1000.0)
    
    # Calcular la fuerza del PID
    # El signo es positivo: si el péndulo se inclina a la derecha (+ángulo),
    # el carro debe moverse a la derecha (+fuerza) para compensar.
    pid_force = (Kp * error) + (Ki * integral) + (Kd * derivative)
    
    previous_error = error

    # --- 3. Gestión de Perturbaciones Manuales ---
    manual_force = 0.0
    key = keyboard.getKey()
    if key == ord('A') or key == ord('a'):
        manual_force = -PERTURBATION_FORCE
    elif key == ord('D') or key == ord('d'):
        manual_force = PERTURBATION_FORCE

    # --- 4. Aplicación de la Fuerza Total ---
    # La fuerza total es la del PID más la perturbación manual.
    total_force = pid_force + manual_force
    
    # Limitar la velocidad para que la simulación sea más estable
    current_velocity = cart_motor.getVelocity()
    if total_force > 0 and current_velocity > MAX_SPEED:
        total_force = 0
    elif total_force < 0 and current_velocity < -MAX_SPEED:
        total_force = 0
        
    cart_motor.setForce(total_force)