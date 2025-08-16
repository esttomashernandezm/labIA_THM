import pygame
import numpy as np
import math

# ==============================================================================
# SECCIÓN 1: VARIABLES CONFIGURABLES
# ==============================================================================
# --- Física del Sistema ---
M, m, L, g = 1.0, 0.2, 0.6, 9.81
I = (1/3) * m * L**2
b = 0.1

# --- Parámetros de Simulación y Control ---
FUERZA_MAG = 10
dt = 0.01
estado_inicial = np.array([0.0, 0.0, math.radians(0.1), 0.0]) # Empezar arriba

# --- Parámetros de Visualización de Vectores ---
FUERZA_VECTOR_ESCALA = 8  # Píxeles por Newton.
COLOR_FUERZA = (0, 180, 0)
COLOR_GRAVEDAD = (220, 0, 0)

# ==============================================================================
# SECCIÓN 2: MODELO DINÁMICO (Sin cambios)
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

def actualizar_estado(estado_actual, fuerza_aplicada, dt):
    derivada = modelo_pendulo_carro(estado_actual, fuerza_aplicada)
    return estado_actual + derivada * dt

# ==============================================================================
# SECCIÓN 3: SIMULACIÓN CON HUD Y VISUALIZACIÓN DE FUERZAS
# ==============================================================================
def ejecutar_sandbox_visual():
    pygame.init()
    ANCHO, ALTO = 1400, 800
    PANTALLA = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("[A/D: Mover | W: Resetear Péndulo | R: Resetear Todo]")
    RELOJ = pygame.time.Clock()
    FUENTE_DATOS = pygame.font.SysFont('Consolas', 22)
    FUENTE_TITULO = pygame.font.SysFont('Consolas', 28, bold=True)
    ESCALA_MUNDO = 250

    estado = np.copy(estado_inicial)

    while True:
        # --- Manejo de Eventos ---
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT: pygame.quit(); return
            if evento.type == pygame.KEYDOWN:
                if evento.key == pygame.K_r: estado = np.copy(estado_inicial)
                if evento.key == pygame.K_w:
                    estado[2], estado[3] = estado_inicial[2], estado_inicial[3]

        # --- Control y Física ---
        teclas = pygame.key.get_pressed()
        fuerza_aplicada = 0.0
        if teclas[pygame.K_a]: fuerza_aplicada = -FUERZA_MAG
        if teclas[pygame.K_d]: fuerza_aplicada = FUERZA_MAG
        
        derivada_actual = modelo_pendulo_carro(estado, fuerza_aplicada)
        estado = estado + derivada_actual * dt
        x, x_dot, theta, _ = estado
        x_ddot = derivada_actual[1]

        # --- LÓGICA DE PAREDES (CORREGIDA) ---
        carro_ancho_px = 80
        pos_carro_px = ANCHO/2 + x * ESCALA_MUNDO
        lim_izq, lim_der = carro_ancho_px/2, ANCHO - carro_ancho_px/2
        
        if pos_carro_px <= lim_izq:
            estado[0] = (lim_izq - ANCHO/2) / ESCALA_MUNDO
            if estado[1] < 0: estado[1] = 0
        elif pos_carro_px >= lim_der:
            estado[0] = (lim_der - ANCHO/2) / ESCALA_MUNDO
            if estado[1] > 0: estado[1] = 0
        
        # --- Dibujado en Pantalla ---
        PANTALLA.fill((245, 245, 245))
        suelo_y = ALTO-150
        pygame.draw.line(PANTALLA, (0,0,0), (0, suelo_y), (ANCHO, suelo_y), 3)

        # Objetos
        pos_carro_px_final = ANCHO/2 + estado[0]*ESCALA_MUNDO
        carro_rect = pygame.Rect(pos_carro_px_final-carro_ancho_px/2, suelo_y-40, carro_ancho_px, 40)
        pygame.draw.rect(PANTALLA, (0,0,139), carro_rect)
        pivote_x, pivote_y = carro_rect.centerx, carro_rect.top
        extremo_x = pivote_x + L*ESCALA_MUNDO*math.sin(theta)
        extremo_y = pivote_y - L*ESCALA_MUNDO*math.cos(theta)
        pygame.draw.line(PANTALLA, (0,0,0), (pivote_x, pivote_y), (extremo_x, extremo_y), 7)
        pygame.draw.circle(PANTALLA, (178,34,34), (int(extremo_x), int(extremo_y)), 12)

        # Dibujar Vectores de Fuerza
        if abs(fuerza_aplicada) > 0:
            longitud_vector_f = fuerza_aplicada * FUERZA_VECTOR_ESCALA
            pygame.draw.line(PANTALLA, COLOR_FUERZA, 
                             (carro_rect.centerx, carro_rect.centery), 
                             (carro_rect.centerx + longitud_vector_f, carro_rect.centery), 5)

        fuerza_g = m * g
        longitud_vector_g = fuerza_g * FUERZA_VECTOR_ESCALA
        pygame.draw.line(PANTALLA, COLOR_GRAVEDAD,
                         (extremo_x, extremo_y),
                         (extremo_x, extremo_y + longitud_vector_g), 5)
        
        # --- Dibujar HUD de Física ---
        # --- CORRECCIÓN SUTIL PARA CLARIDAD DEL ÁNGULO ---
        # Normalizamos el ángulo a un rango de -pi a +pi antes de mostrarlo.
        # Esto no afecta la física, solo la visualización para que sea más intuitiva.
        angulo_visual = (theta + np.pi) % (2 * np.pi) - np.pi
        
        hud_items = [
            ("--- Carro ---", ""), 
            ("Posición (x)", f"{x:8.2f} m"),
            ("Velocidad (ẋ)", f"{x_dot:8.2f} m/s"), 
            ("Aceleración (ẍ)", f"{x_ddot:8.2f} m/s²"),
            ("", ""), 
            ("--- Péndulo ---", ""), 
            ("Ángulo (θ)", f"{math.degrees(angulo_visual):8.2f} °"), # Usamos la variable normalizada
            ("", ""), 
            ("--- Control ---", ""), 
            ("Fuerza Aplicada (F)", f"{fuerza_aplicada:8.2f} N")
        ]

        PANTALLA.blit(FUENTE_TITULO.render("Datos Físicos en Vivo", True, (0,0,0)), (10, 10))
        for i, (label, value) in enumerate(hud_items):
            texto = f"{label:<20} {value:>12}"
            render_texto = FUENTE_DATOS.render(texto, True, (0,0,0))
            PANTALLA.blit(render_texto, (10, 50 + i * 25))

        pygame.display.flip()
        RELOJ.tick(1 / dt)

if __name__ == "__main__":
    ejecutar_sandbox_visual()