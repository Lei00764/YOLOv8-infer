# @time   : 2023/7/11 23:33
# @author : Xiang Lei

import os

def rename_images(folder_path):
    # 获取文件夹中的所有文件
    files = os.listdir(folder_path)

    # 用于计数重命名文件
    count = 1

    # 遍历文件夹中的所有文件
    for file in files:
        # 确定文件的完整路径
        file_path = os.path.join(folder_path, file)

        # 检查文件是否为图片文件
        if os.path.isfile(file_path) and file.endswith(('.jpg', '.jpeg', '.png', '.gif')):
            # 构建新的文件名
            new_file_name = str(count) + '.jpg'
            new_file_path = os.path.join(folder_path, new_file_name)

            # 重命名文件
            os.rename(file_path, new_file_path)

            # 增加计数器
            count += 1

    print("重命名完成。")

# 指定图片文件夹的路径
folder_path = "images"

# 调用函数进行重命名
rename_images(folder_path)
