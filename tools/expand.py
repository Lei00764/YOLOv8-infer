import cv2
import numpy as np
import random
import math
import os

def rotate_image(image, angle):
    # 获取旋转的中心点
    height, width = image.shape[:2]
    image_center = (width/2, height/2)
    
    # 计算旋转角度的弧度
    angle_rad = math.radians(angle)
    
    # 计算新图像的尺寸
    new_width = abs(np.sin(angle_rad)*height) + abs(np.cos(angle_rad)*width)
    new_height = abs(np.sin(angle_rad)*width) + abs(np.cos(angle_rad)*height)

    # 获取旋转矩阵
    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1)
    
    # 更新旋转矩阵的偏移量
    rotation_mat[0, 2] += (new_width - width) / 2
    rotation_mat[1, 2] += (new_height - height) / 2

    # 应用旋转
    rotated_img = cv2.warpAffine(image, rotation_mat, (int(new_width), int(new_height)))
    return rotated_img

# 加载模板图片
template_img = cv2.imread('mask1.png', cv2.IMREAD_UNCHANGED)

output_folder = "expand_image"

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

for i in range(150, 200):
    # 随机生成旋转角度
    angle = random.uniform(-30, 30)

    # 对图片进行旋转
    rotated_img = rotate_image(template_img, angle)

    # 获取旋转后的图片尺寸
    rotated_height, rotated_width = rotated_img.shape[:2]

    # 生成随机浅灰色背景图片
    background_img = np.random.randint(140, 180, (rotated_height, rotated_width, 3), dtype=np.uint8)

    # 提取遮罩图像的Alpha通道
    mask_alpha = rotated_img[:, :, 3]

    # 将Alpha通道转换为3通道
    mask_alpha = cv2.cvtColor(mask_alpha, cv2.COLOR_GRAY2BGR)

    # 将Alpha通道进行归一化
    mask_alpha = mask_alpha.astype(float) / 255

    # 将模板图像的RGB通道与Alpha通道进行分离
    rotated_rgb = rotated_img[:, :, :3]

    # 对背景图像和模板图像进行透明融合
    result_img = (1 - mask_alpha) * background_img + rotated_rgb * mask_alpha
    result_img = result_img.astype(np.uint8)

    # 计算新的尺寸
    new_height = int(rotated_height * 2)
    new_width = int(rotated_width * 2)

    # 创建随机浅灰色的新背景图片
    extended_img = np.random.randint(140, 180, (new_height, new_width, 3), dtype=np.uint8)

    # 将融合后的图片放置到新的背景图片的随机位置
    start_y = random.randint(0, new_height - rotated_height)
    start_x = random.randint(0, new_width - rotated_width)
    extended_img[start_y:start_y+rotated_height, start_x:start_x+rotated_width] = result_img

    # 生成文件名并保存图片
    filename = os.path.join(output_folder, 'output_{:03d}.png'.format(i))
    cv2.imwrite(filename, extended_img)

print("Images generated and saved to '{}'.".format(output_folder))
