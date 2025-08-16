from controller import Robot
import time
import os

# --- PARÁMETROS Y GANANCIAS PID ---
# ¡Aquí puedes editar las ganancias para ajustar el controlador!
Kp = 35 #35
Ki = 0.5
Kd = 8.0

# --- INICIALIZACIÓN DEL ROBOT Y TIMESTEP ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- OBTENCIÓN DE DISPOSITIVOS DEL ROBOT ---
# Los nombres "cart motor" y "pole position sensor" deben coincidir
# con los nombres definidos en el archivo .wbt del mundo.
cart_motor = robot.getDevice("cart motor")
pole_sensor = robot.getDevice("pole position sensor")
pole_sensor.enable(timestep)

# --- CONFIGURACIÓN DEL SEGUIMIENTO DE TIEMPO (DE TU CÓDIGO) ---
start_time = time.time()
best_time_file = "best_time.txt"

def load_best_time():
    if os.path.exists(best_time_file):
        with open(best_time_file, "r") as f:
            # Añadimos un try-except por si el archivo está vacío o corrupto
            try:
                return float(f.read().strip())
            except ValueError:
                return 0.0
    return 0.0

def save_best_time(t):
    with open(best_time_file, "w") as f:
        f.write(f"{t:.2f}")

best_time = load_best_time()
print(f"🔥 Mejor tiempo registrado: {best_time:.2f} s")

# --- INICIALIZACIÓN DE VARIABLES DEL PID ---
integral = 0.0
previous_error = 0.0

# --- BUCLE PRINCIPAL DE SIMULACIÓN ---
while robot.step(timestep) != -1:
    # --- Lógica de seguimiento de tiempo (de tu código) ---
    elapsed = time.time() - start_time
    print(f"⏱ Tiempo transcurrido: {elapsed:.2f} s", end="\r")

    # ==========================================================
    # Aquí implementamos el control del péndulo (lógica PID)
    # ==========================================================
    
    # 1. Lectura de sensores
    pole_angle = pole_sensor.getValue()  # Ángulo del péndulo en radianes

    # 2. Cálculo del error
    error = pole_angle

    # 3. Cálculo de los términos PID
    proportional = Kp * error
    integral += error * (timestep / 1000.0)
    derivative = (error - previous_error) / (timestep / 1000.0)
    previous_error = error

    # 4. Cálculo de la salida (fuerza a aplicar)
    output = proportional + Ki * integral + Kd * derivative
    
    # 5. Aplicación de la fuerza al motor
    cart_motor.setForce(output)

    # ==========================================================

    # --- Lógica de guardado de récord (de tu código) ---
    if elapsed > best_time:
        # Imprimimos en una nueva línea para no sobreescribir el contador
        print(f"\n🏆 ¡Nuevo récord! {elapsed:.2f} s (El anterior era {best_time:.2f} s)")
        save_best_time(elapsed)
        best_time = elapsed