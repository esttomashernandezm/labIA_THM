import pygame
import numpy as np
import math

# ==============================================================================
# SECCIÓN 1: CONTROLADOR DIFUSO (Sin cambios)
# ==============================================================================
class ControladorDifuso:
    """
    Controlador difuso Mamdani con inferencia Max-Min para el péndulo invertido.
    No utiliza ninguna biblioteca externa de lógica difusa.
    """
    def __init__(self):
        # --- 1. Definir universos de discurso (rangos de operación) ---
        self.universo_angulo = [-0.8, 0.8]  # rad (aprox. -45 a +45 grados)
        self.universo_vel_angular = [-3, 3] # rad/s
        self.universo_fuerza = [-15, 15]   # Newtons (coincide con FUERZA_MAG)

        # --- 2. Definir los conjuntos difusos de ENTRADA (Membresía Trapezoidal) ---
        self.mf_angulo = {
            'amn': [-0.8, -0.8, -0.5, -0.3], 'an': [-0.5, -0.3, -0.2, 0.0],
            'ap': [0.0, 0.2, 0.3, 0.5], 'amp': [0.3, 0.5, 0.8, 0.8]
        }
        self.mf_vel_angular = { 'vn': [-3.0, -3.0, -1.0, 0.0], 'vp': [0.0, 1.0, 3.0, 3.0] }

        # --- 3. Definir los conjuntos difusos de SALIDA (Membresía Trapezoidal) ---
        self.mf_fuerza = {
            'xmn': [-15, -15, -12, -10], 'xn': [-12, -10, -8, -5],
            'x0': [-5, -2, 2, 5], 'xp': [5, 8, 10, 12], 'xmp': [10, 12, 15, 15]
        }
        self.puntos_salida = np.linspace(self.universo_fuerza[0], self.universo_fuerza[1], 100)

    def _funcion_membresia_trapezoidal(self, x, params):
        a, b, c, d = params
        if x <= a or x >= d: return 0.0
        elif a < x < b: return (x - a) / (b - a) if b != a else 1.0
        elif b <= x <= c: return 1.0
        elif c < x < d: return (d - x) / (d - c) if d != c else 1.0
        return 0.0

    def calcular_fuerza(self, angulo, vel_angular):
        memb_angulo = {k: self._funcion_membresia_trapezoidal(angulo, p) for k, p in self.mf_angulo.items()}
        memb_vel = {k: self._funcion_membresia_trapezoidal(vel_angular, p) for k, p in self.mf_vel_angular.items()}

        fuerza_r1 = min(memb_angulo['ap'], memb_vel['vp'])
        fuerza_r2 = min(memb_angulo['ap'], memb_vel['vn'])
        fuerza_r3 = min(memb_angulo['an'], memb_vel['vp'])
        fuerza_r4 = min(memb_angulo['an'], memb_vel['vn'])
        fuerza_r5 = min(memb_angulo['amp'], memb_vel['vp'])
        fuerza_r6 = min(memb_angulo['amp'], memb_vel['vn'])
        fuerza_r7 = min(memb_angulo['amn'], memb_vel['vp'])
        fuerza_r8 = min(memb_angulo['amn'], memb_vel['vn'])

        fuerza_activacion = {
            'xmp': fuerza_r5, 'xp': max(fuerza_r1, fuerza_r6),
            'x0': max(fuerza_r2, fuerza_r3), 'xn': max(fuerza_r4, fuerza_r7),
            'xmn': fuerza_r8
        }
        
        superficie_agregada = np.zeros_like(self.puntos_salida)
        for i, z in enumerate(self.puntos_salida):
            max_grado_membresia = 0.0
            for consecuente, fuerza_regla in fuerza_activacion.items():
                grado_memb = self._funcion_membresia_trapezoidal(z, self.mf_fuerza[consecuente])
                membresia_activada = min(fuerza_regla, grado_memb)
                if membresia_activada > max_grado_membresia:
                    max_grado_membresia = membresia_activada
            superficie_agregada[i] = max_grado_membresia

        numerador = np.sum(self.puntos_salida * superficie_agregada)
        denominador = np.sum(superficie_agregada)
        return 0.0 if abs(denominador) < 1e-6 else numerador / denominador

# ==============================================================================
# SECCIÓN 2: VARIABLES CONFIGURABLES (Sin cambios)
# ==============================================================================
M, m, L, g = 1.0, 0.2, 0.6, 9.81
I = (1/3) * m * L**2
b = 0.1
FUERZA_MAG = 15.0
FUERZA_PERTURBACION_MAG = 25
dt = 0.01
estado_inicial = np.array([0.0, 0.0, math.radians(1.5), 0.0])
FUERZA_VECTOR_ESCALA = 6
COLOR_FUERZA = (0, 180, 0)
COLOR_GRAVEDAD = (220, 0, 0)

# ==============================================================================
# SECCIÓN 3: MODELO DINÁMICO (Sin cambios)
# ==============================================================================
def modelo_pendulo_carro(estado_actual, fuerza_aplicada):
    _, _, theta, theta_dot = estado_actual
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    A, B = M + m, fuerza_aplicada + m * L * (theta_dot**2) * sin_t
    den = A * (I + m * L**2) - (m * L * cos_t)**2
    if abs(den) < 1e-6: return np.array([0, 0, 0, 0])
    theta_ddot = ((A*m*g*L*sin_t) - (m*L*cos_t*B) - (A*b*theta_dot)) / den
    x_ddot = ((B*(I+m*L**2)) - (m*L*cos_t)*(m*g*L*sin_t-b*theta_dot)) / den
    return np.array([estado_actual[1], x_ddot, estado_actual[3], theta_ddot])

# ==============================================================================
# SECCIÓN 4: SIMULACIÓN CON CONTROLADOR DIFUSO Y PERTURBACIONES
# ==============================================================================
def ejecutar_sandbox_visual():
    pygame.init()
    ANCHO, ALTO = 1400, 800
    PANTALLA = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("[A/D: Perturbar | W: Reset Péndulo | R: Reset Total]")
    RELOJ = pygame.time.Clock()
    FUENTE_DATOS = pygame.font.SysFont('Consolas', 22)
    FUENTE_TITULO = pygame.font.SysFont('Consolas', 28, bold=True)
    ESCALA_MUNDO = 250

    estado = np.copy(estado_inicial)
    controlador_difuso = ControladorDifuso()

    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT: pygame.quit(); return
            if evento.type == pygame.KEYDOWN:
                if evento.key == pygame.K_r: estado = np.copy(estado_inicial)
                if evento.key == pygame.K_w: estado[2], estado[3] = estado_inicial[2], estado_inicial[3]

        teclas = pygame.key.get_pressed()
        fuerza_perturbacion = 0.0
        if teclas[pygame.K_a]: fuerza_perturbacion = -FUERZA_PERTURBACION_MAG
        if teclas[pygame.K_d]: fuerza_perturbacion = FUERZA_PERTURBACION_MAG

        x, x_dot, theta, theta_dot = estado
        fuerza_controlador = controlador_difuso.calcular_fuerza(theta, theta_dot)
        fuerza_aplicada = fuerza_controlador + fuerza_perturbacion
        fuerza_aplicada = np.clip(fuerza_aplicada, -FUERZA_MAG, FUERZA_MAG)

        derivada_actual = modelo_pendulo_carro(estado, fuerza_aplicada)
        estado = estado + derivada_actual * dt
        x_ddot = derivada_actual[1]

        # --- ¡¡¡MODIFICADO!!! LÓGICA DE PAREDES CON REBOTE ---
        COEF_RESTITUCION = 0.5  # Factor de rebote (0=no rebota, 1=rebote perfecto)
        carro_ancho_px = 80
        pos_carro_px = ANCHO / 2 + estado[0] * ESCALA_MUNDO
        lim_izq, lim_der = carro_ancho_px / 2, ANCHO - carro_ancho_px / 2

        # Si se pasa del límite IZQUIERDO y se está MOVIENDO hacia la izquierda
        if pos_carro_px <= lim_izq and estado[1] < 0:
            estado[0] = (lim_izq - ANCHO / 2) / ESCALA_MUNDO  # Reposicionar en el borde
            estado[1] *= -COEF_RESTITUCION  # Invertir y amortiguar la velocidad (rebote)
        
        # Si se pasa del límite DERECHO y se está MOVIENDO hacia la derecha
        elif pos_carro_px >= lim_der and estado[1] > 0:
            estado[0] = (lim_der - ANCHO / 2) / ESCALA_MUNDO  # Reposicionar en el borde
            estado[1] *= -COEF_RESTITUCION  # Invertir y amortiguar la velocidad (rebote)
        
        # --- Dibujado en Pantalla (sin cambios) ---
        PANTALLA.fill((245, 245, 245))
        suelo_y = ALTO - 150
        pygame.draw.line(PANTALLA, (0, 0, 0), (0, suelo_y), (ANCHO, suelo_y), 3)

        pos_carro_px_final = ANCHO / 2 + estado[0] * ESCALA_MUNDO
        carro_rect = pygame.Rect(pos_carro_px_final - carro_ancho_px / 2, suelo_y - 40, carro_ancho_px, 40)
        pygame.draw.rect(PANTALLA, (0, 0, 139), carro_rect)
        pivote_x, pivote_y = carro_rect.centerx, carro_rect.top
        extremo_x = pivote_x + L * ESCALA_MUNDO * math.sin(theta)
        extremo_y = pivote_y - L * ESCALA_MUNDO * math.cos(theta)
        pygame.draw.line(PANTALLA, (0, 0, 0), (pivote_x, pivote_y), (extremo_x, extremo_y), 7)
        pygame.draw.circle(PANTALLA, (178, 34, 34), (int(extremo_x), int(extremo_y)), 12)

        if abs(fuerza_aplicada) > 0.01:
            long_vector_f = fuerza_aplicada * FUERZA_VECTOR_ESCALA
            pygame.draw.line(PANTALLA, COLOR_FUERZA, (carro_rect.centerx, carro_rect.centery), (carro_rect.centerx + long_vector_f, carro_rect.centery), 5)
        
        fuerza_g = m * g
        long_vector_g = fuerza_g * FUERZA_VECTOR_ESCALA
        pygame.draw.line(PANTALLA, COLOR_GRAVEDAD, (extremo_x, extremo_y), (extremo_x, extremo_y + long_vector_g), 5)
        
        hud_items = [
            ("--- Carro ---", ""), ("Posición (x)", f"{estado[0]:8.2f} m"),
            ("Velocidad (ẋ)", f"{estado[1]:8.2f} m/s"), ("Aceleración (ẍ)", f"{x_ddot:8.2f} m/s²"),
            ("", ""), ("--- Péndulo ---", ""), ("Ángulo (θ)", f"{math.degrees(theta):8.2f} °"),
            ("Vel. Angular (θ̇)", f"{theta_dot:8.2f} rad/s"), ("", ""),
            ("--- Controlador Difuso ---", ""), ("Perturbación (A/D)", f"{fuerza_perturbacion:8.2f} N"),
            ("Fuerza Difusa Calc.", f"{fuerza_controlador:8.2f} N"), ("Fuerza TOTAL Aplic.", f"{fuerza_aplicada:8.2f} N"),
        ]

        PANTALLA.blit(FUENTE_TITULO.render("Datos Físicos y Control", True, (0, 0, 0)), (10, 10))
        for i, (label, value) in enumerate(hud_items):
            texto = f"{label:<20} {value:>12}"
            PANTALLA.blit(FUENTE_DATOS.render(texto, True, (0, 0, 0)), (10, 50 + i * 25))

        pygame.display.flip()
        RELOJ.tick(1 / dt)

if __name__ == "__main__":
    ejecutar_sandbox_visual()