import argparse
import collections
import logging
import os
import sys
import re
import time
import timeit
from matplotlib import pyplot as plt

import pyecore.resources
from ecore import EcoreGenerator

URL_PATTERN = re.compile('^http(s)?://.*')


def main():
    source_file = input("Enter input file:")
    output_path = input("Enter output path:")
    if(os.path.exists(source_file) and os.path.exists(output_path)):
        model = load_model(source_file)
        EcoreGenerator().generate(model, output_path)
    else:
        print("paths not found")

def select_uri_implementation(ecore_model_path):
    """Select the right URI implementation regarding the Ecore model path schema."""
    if URL_PATTERN.match(ecore_model_path):
        return pyecore.resources.resource.HttpURI
    return pyecore.resources.URI

def load_model(ecore_model_path):
    """Load a single Ecore model and return the root package."""
    rset = pyecore.resources.ResourceSet()
    uri_implementation = select_uri_implementation(ecore_model_path)
    resource = rset.get_resource(uri_implementation(ecore_model_path))
    return resource.contents[0]

if __name__ == '__main__':  # nocover
    #executiontime Calculator
    
    # font = {'family': 'normal',
    #         'color':  'black',
    #         'weight': 'normal',
    #         'size': 14,
    #         }
    # y = []
    # for n in range(100):
    #     start = time.time()
        main()
    #     end= time.time()
    #     y.append(end-start)
    # plt.boxplot(y, vert=False)
    # #plt.title("Ausführungszeit des Generators", fontdict=font)
    # plt.xlabel('Zeit in s', fontdict=font)
    # plt.show()
    # print("Durchschnitt: " + str(sum(y)/100))
    
