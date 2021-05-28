import statsmodels.api as sm
import cv2
import numpy as np
low = np.array([25, 150, 150])
high = np.array([35, 255, 255])

def segment_yellow_line(hsv):
    """
    Segmenta a cor amarela da pista e devolve a máscara
    :param hsv  : imagem colorida em formato HSV
    :return mask: mascara que destaca apenas amarelo 
    """
    mask = cv2.inRange(hsv, low, high)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    gray = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    return gray

def ajuste_linear_x_fy(mask):
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        y = coef_angular*x + coef_linear
    """ 
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0]
    yimg_c = sm.add_constant(yimg)
    model = sm.OLS(ximg,yimg_c)
    results = model.fit()
    coef_angular = results.params[1] # Pegamos o beta 1
    coef_linear =  results.params[0] # Pegamso o beta 0
    return coef_angular, coef_linear

def ajuste_linear_grafico_x_fy(mask):
    """Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem"""
    coef_angular, coef_linear = ajuste_linear_x_fy(mask)
    pontos = np.where(mask==255) # esta linha é pesada e ficou redundante
    ximg = pontos[1]
    yimg = pontos[0]
    y_bounds = np.array([min(yimg), max(yimg)])
    x_bounds = coef_angular*y_bounds + coef_linear
    x_int = x_bounds.astype(dtype=np.int64)
    y_int = y_bounds.astype(dtype=np.int64)
    cv2.line(mask, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=3);    
    return x_bounds, y_bounds