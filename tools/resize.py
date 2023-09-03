import cv2
import numpy as np
import os
import glob

def crop_and_resize_image(image, target_size):
    # 获取图片尺寸
    height, width = image.shape[:2]

    # 计算裁剪尺寸
    size = min(height, width)

    # 计算裁剪的起点
    start_x = (width - size) // 2
    start_y = (height - size) // 2

    # 裁剪图片
    cropped_img = image[start_y:start_y+size, start_x:start_x+size]

    # 缩放图片
    resized_img = cv2.resize(cropped_img, (target_size, target_size), interpolation=cv2.INTER_AREA)

    return resized_img

input_folder = "images"
output_folder = "resize_images_2"

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 获取输入文件夹中的所有图片
image_files = glob.glob(os.path.join(input_folder, '*.jpeg'))

for i, image_file in enumerate(image_files):
    # 读取图片
    img = cv2.imread(image_file)

    # 裁剪并缩放图片
    result_img = crop_and_resize_image(img, 416)

    # 生成文件名并保存图片
    filename = os.path.join(output_folder, 'output_{:03d}.png'.format(i))
    cv2.imwrite(filename, result_img)

print("Images cropped and saved to '{}'.".format(output_folder))
