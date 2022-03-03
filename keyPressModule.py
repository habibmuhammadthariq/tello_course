import pygame

def init():
    pygame.init()
    win = pygame.display.set_mode((300, 300))

def getKey(keyName):
    ans = False

    for eve in pygame.event.get(): pass
    # get the key input from the user
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))

    # check the similarity between key input with the requirement key
    if keyInput[myKey]:
        ans = True
    # update pygame
    pygame.display.update()

    return ans

def main():
    if getKey("LEFT"): print("Left Key Pressed")
    if getKey("RIGHT"): print("Right Key Pressed")

if __name__ == '__main__':
    init()
    while True:
        main()
