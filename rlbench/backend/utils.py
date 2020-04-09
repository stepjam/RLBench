"""Creates an image from a numpy array of floating point depth data.

Taken from https://github.com/ahundt/robotics_setup/blob/master/datasets/google_brain_robot_data/depth_image_encoding.py

Examples:

  depth_array is a 2D numpy array of floating point depth data in meters.

  depth_rgb = FloatArrayToRgbImage(depth_array)
  depth_rgb is a PIL Image object containing the same data as 24-bit
  integers encoded in the RGB bytes.
  depth_rgb.save('image_file.png') - to save to a file.

  depth_array2 = ImageToFloatArray(depth_rgb)
  depth_array2 is a 2D numpy array containing the same data as
  depth_array up to the precision of the RGB image (1/256 mm).

  depth_gray = FloatArrayToGrayImage(depth_array)
  depth_gray is a PIL Image object containing the same data rounded to
  8-bit integers.
  depth_gray.save('image_file.jpg', quality=95) - to save to a file.

  depth_array3 = ImageToFloatArray(depth_gray)
  depth_array3 is a 2D numpy array containing the same data as
  depth_array up to the precision of the grayscale image (1 cm).

The image conversions first scale and round the values and then pack
them into the desired type in a numpy array before converting the
array to a PIL Image object.  The Image can be saved in any format.
We are using PNG for RGB and high quality JPEG for grayscale images.

You can use different numeric types (e.g. np.uint16, np.int32), but
not all combinations of numeric type and image format are supported by
PIL or standard image viewers.

"""

import numpy as np
from PIL import Image


def ClipFloatValues(float_array, min_value, max_value):
  """Clips values to the range [min_value, max_value].

  First checks if any values are out of range and prints a message.
  Then clips all values to the given range.

  Args:
    float_array: 2D array of floating point values to be clipped.
    min_value: Minimum value of clip range.
    max_value: Maximum value of clip range.

  Returns:
    The clipped array.

  """
  if float_array.min() < min_value or float_array.max() > max_value:
    float_array = np.clip(float_array, min_value, max_value)
  return float_array


DEFAULT_RGB_SCALE_FACTOR = 256000.0


def float_array_to_rgb_image(float_array,
                             scale_factor=DEFAULT_RGB_SCALE_FACTOR,
                             drop_blue=False):
  """Convert a floating point array of values to an RGB image.

  Convert floating point values to a fixed point representation where
  the RGB bytes represent a 24-bit integer.
  R is the high order byte.
  B is the low order byte.
  The precision of the depth image is 1/256 mm.

  Floating point values are scaled so that the integer values cover
  the representable range of depths.

  This image representation should only use lossless compression.

  Args:
    float_array: Input array of floating point depth values in meters.
    scale_factor: Scale value applied to all float values.
    drop_blue: Zero out the blue channel to improve compression, results in 1mm
      precision depth values.

  Returns:
    24-bit RGB PIL Image object representing depth values.
  """
  # Scale the floating point array.
  scaled_array = np.floor(float_array * scale_factor + 0.5)
  # Convert the array to integer type and clip to representable range.
  min_inttype = 0
  max_inttype = 2**24 - 1
  scaled_array = ClipFloatValues(scaled_array, min_inttype, max_inttype)
  int_array = scaled_array.astype(np.uint32)
  # Calculate:
  #   r = (f / 256) / 256  high byte
  #   g = (f / 256) % 256  middle byte
  #   b = f % 256          low byte
  rg = np.divide(int_array, 256)
  r = np.divide(rg, 256)
  g = np.mod(rg, 256)
  image_shape = int_array.shape
  rgb_array = np.zeros((image_shape[0], image_shape[1], 3), dtype=np.uint8)
  rgb_array[..., 0] = r
  rgb_array[..., 1] = g
  if not drop_blue:
    # Calculate the blue channel and add it to the array.
    b = np.mod(int_array, 256)
    rgb_array[..., 2] = b
  image_mode = 'RGB'
  image = Image.fromarray(rgb_array, mode=image_mode)
  return image


DEFAULT_GRAY_SCALE_FACTOR = {np.uint8: 100.0,
                             np.uint16: 1000.0,
                             np.int32: DEFAULT_RGB_SCALE_FACTOR}


def float_array_to_grayscale_image(float_array, scale_factor=None, image_dtype=np.uint8):
  """Convert a floating point array of values to an RGB image.

  Convert floating point values to a fixed point representation with
  the given bit depth.

  The precision of the depth image with default scale_factor is:
    uint8: 1cm, with a range of [0, 2.55m]
    uint16: 1mm, with a range of [0, 65.5m]
    int32: 1/256mm, with a range of [0, 8388m]

  Right now, PIL turns uint16 images into a very strange format and
  does not decode int32 images properly.  Only uint8 works correctly.

  Args:
    float_array: Input array of floating point depth values in meters.
    scale_factor: Scale value applied to all float values.
    image_dtype: Image datatype, which controls the bit depth of the grayscale
      image.

  Returns:
    Grayscale PIL Image object representing depth values.

  """
  # Ensure that we have a valid numeric type for the image.
  if image_dtype == np.uint16:
    image_mode = 'I;16'
  elif image_dtype == np.int32:
    image_mode = 'I'
  else:
    image_dtype = np.uint8
    image_mode = 'L'
  if scale_factor is None:
    scale_factor = DEFAULT_GRAY_SCALE_FACTOR[image_dtype]
  # Scale the floating point array.
  scaled_array = np.floor(float_array * scale_factor + 0.5)
  # Convert the array to integer type and clip to representable range.
  min_dtype = np.iinfo(image_dtype).min
  max_dtype = np.iinfo(image_dtype).max
  scaled_array = ClipFloatValues(scaled_array, min_dtype, max_dtype)

  image_array = scaled_array.astype(image_dtype)
  image = Image.fromarray(image_array, mode=image_mode)
  return image


def image_to_float_array(image, scale_factor=None):
  """Recovers the depth values from an image.

  Reverses the depth to image conversion performed by FloatArrayToRgbImage or
  FloatArrayToGrayImage.

  The image is treated as an array of fixed point depth values.  Each
  value is converted to float and scaled by the inverse of the factor
  that was used to generate the Image object from depth values.  If
  scale_factor is specified, it should be the same value that was
  specified in the original conversion.

  The result of this function should be equal to the original input
  within the precision of the conversion.

  Args:
    image: Depth image output of FloatArrayTo[Format]Image.
    scale_factor: Fixed point scale factor.

  Returns:
    A 2D floating point numpy array representing a depth image.

  """
  image_array = np.array(image)
  image_dtype = image_array.dtype
  image_shape = image_array.shape

  channels = image_shape[2] if len(image_shape) > 2 else 1
  assert 2 <= len(image_shape) <= 3
  if channels == 3:
    # RGB image needs to be converted to 24 bit integer.
    float_array = np.sum(image_array * [65536, 256, 1], axis=2)
    if scale_factor is None:
      scale_factor = DEFAULT_RGB_SCALE_FACTOR
  else:
    if scale_factor is None:
      scale_factor = DEFAULT_GRAY_SCALE_FACTOR[image_dtype.type]
    float_array = image_array.astype(np.float32)
  scaled_array = float_array / scale_factor
  return scaled_array


def task_file_to_task_class(task_file):
  import importlib
  name = task_file.replace('.py', '')
  class_name = ''.join([w[0].upper() + w[1:] for w in name.split('_')])
  mod = importlib.import_module("rlbench.tasks.%s" % name)
  mod = importlib.reload(mod)
  task_class = getattr(mod, class_name)
  return task_class


def rgb_handles_to_mask(rgb_coded_handles):
  # rgb_coded_handles should be (w, h, c)
  # Handle encoded as : handle = R + G * 256 + B * 256 * 256
  rgb_coded_handles *= 255  # takes rgb range to 0 -> 255
  rgb_coded_handles.astype(int)
  return (rgb_coded_handles[:, :, 0] +
          rgb_coded_handles[:, :, 1] * 256 +
          rgb_coded_handles[:, :, 2] * 256 * 256)
