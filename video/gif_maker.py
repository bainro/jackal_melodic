# Makes a gif from files

import imageio
from tqdm import tqdm
def make_gif(files, output_file):
    images = []
    for file in tqdm(files):
        images.append(imageio.imread(file))
    imageio.mimsave(output_file, images, format='GIF', fps=10)

if __name__ == "__main__":
    files = [f'video/demo_gif/{i}.png' for i in range(351)]
    make_gif(files, "video/demo.gif")