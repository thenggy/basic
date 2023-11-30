import numpy as np
from skimage.transform import downscale_local_mean

# Larger input array
larger_input_array = np.array([[1, 2, 3, 4, 5],
                               [6, 7, 8, 9, 10],
                               [11, 12, 13, 14, 15],
                               [16, 17, 18, 19, 20],
                               [21, 22, 23, 24, 25]])

# Define scaling factors
larger_scale_factors = (2, 2)  # (rows, columns)

# Apply downscaling using local mean
larger_downscaled_array = downscale_local_mean(larger_input_array, larger_scale_factors)

print("Larger Original array:")
print(larger_input_array)

print("\nLarger Downscaled array:")
print(larger_downscaled_array)
