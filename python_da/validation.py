from BCI2000Tools.FileReader import bcistream
import mne
import numpy as np



# print('hello world')

stream = bcistream("NameS001R02.dat")

print(stream.params.keys())
print(stream.states.keys())

print(stream.states.)