import cv2
import numpy as np
import os

class FramePreprocessor:
    def __init__(self, config=None):
        """
        config (dict):
            - target_width: Ancho estándar (def: 640)
            - blur_kernel: Suavizado para quitar ruido (def: 5)
            - to_gray: Convertir a blanco y negro (def: False)
        """
        self.cfg = config if config else {}
        self.target_width = self.cfg.get('target_width', 640)
        self.blur_k = self.cfg.get('blur_kernel', 5)
        self.to_gray = self.cfg.get('to_gray', False)

    def _decode_image(self, input_data):
        """
        Método interno helper: Intenta convertir 'input_data' en una matriz de OpenCV.
        """
        # CASO 1: Es una matriz de Numpy (ya es una imagen cargada o frame de video)
        if isinstance(input_data, np.ndarray):
            return input_data

        # CASO 2: Son bytes crudos (lo que llega de una petición HTTP/Web)
        elif isinstance(input_data, (bytes, bytearray)):
            # Convertimos bytes a array de numpy y decodificamos
            np_arr = np.frombuffer(input_data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("No se pudo decodificar la imagen desde los bytes.")
            return image

        # CASO 3: Es una ruta de archivo (string)
        elif isinstance(input_data, str):
            if not os.path.exists(input_data):
                raise FileNotFoundError(f"No se encontró el archivo: {input_data}")
            return cv2.imread(input_data)

        else:
            raise TypeError(f"Tipo de dato no soportado: {type(input_data)}")

    def process(self, raw_input):
        """
        Recibe: Bytes, Ruta de archivo o Matriz Numpy.
        Retorna: (imagen_estandarizada, factor_de_escala)
        """
        try:
            # 1. Estandarizar entrada a matriz
            image = self._decode_image(raw_input)
            
            # 2. Calcular nueva geometría
            (h, w) = image.shape[:2]
            
            # Evitamos dividir por cero o procesar imágenes vacías
            if w == 0 or h == 0:
                return None, 0

            scale_factor = self.target_width / float(w)
            new_height = int(h * scale_factor)

            # 3. Redimensionar (Standardizing)
            # INTER_AREA es excelente para reducir tamaño sin perder detalles (aliasing)
            processed_img = cv2.resize(image, (self.target_width, new_height), interpolation=cv2.INTER_AREA)

            # 4. Limpieza (Denoising)
            if self.blur_k > 0:
                # Asegurar que el kernel sea impar
                k = self.blur_k if self.blur_k % 2 == 1 else self.blur_k + 1
                processed_img = cv2.GaussianBlur(processed_img, (k, k), 0)

            # 5. Formato de Color
            if self.to_gray:
                processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2GRAY)

            return processed_img, scale_factor

        except Exception as e:
            # Aquí podrías loggear el error
            print(f"[FramePreprocessor Error]: {e}")
            return None, 0
