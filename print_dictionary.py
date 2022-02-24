import pprint
import pickle
import sys

def main():
  
    inFile = sys.argv[1]

    with open(inFile, 'rb') as cached_pcd_file:
        cache_data = pickle.load(cached_pcd_file)
        pprint.pprint(cache_data)
    return

if __name__ == "__main__":
    main()
