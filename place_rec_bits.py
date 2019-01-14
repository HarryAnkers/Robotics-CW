#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import sweep
import math

# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 72):
        self.sig = [0] * no_bins

    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];

        # Fills the filenames variable with names like loc_%%.dat
        # where %% are 2 digits (00, 01, 02...) indicating the location number.
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1

        if (n >= self.size):
            return -1;
        else:
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])

    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)

        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    s = s.rstrip("\n")
                    ls.sig[i] = float(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."

        return ls

# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls):
    sweep_val = sweep.sweep()
    print("finished sweep")

    for i in range(len(ls.sig)):
        ls.sig[i] = sweep_val[i]

# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    dist = 0

    for i in range(len(ls1.sig)):
        diff = ls1.sig[i] - ls2.sig[i]
        abs_diff = diff ** 2
        dist += abs_diff

    return dist


# This function characterizes the current location, and stores the obtained
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    print("CHARACTERRISED LOCATION")
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return

    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = LocationSignature();
    # ls_obs stores signature for current location
    characterize_location(ls_obs);

    min_dist = 100000000000000
    min_loc = 0

    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx);
        dist = compare_signatures(ls_obs, ls_read)

        if dist < min_dist:
            min_dist = dist
            min_loc = idx

    print('Min location is : ' + str(min_loc))
    return min_loc

def recognize_orientation(ls_compare):
    ls_obs = LocationSignature()
    characterize_location(ls_obs)

    min_angle_diff = 100000000000
    num_shift = 0
    
    for i in range(72):
        ls_shifted = ls_obs[1:]+[ls_obs[-1]] # Shift target signature by one
        dist = compare_signatures(ls_shifted, ls_compare)
        if dist < min_angle_diff:
            # Update minimum difference and corresponding angle
            min_angle_diff = dist
            num_shift = i
    print('minimum angle difference: ' + str(min_angle_diff))
    print('shifted angle: ' + str((num_shift * 5) * (math.pi / 180)))
    return (num_shift * 5) * (math.pi / 180)

# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files().
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

signatures = SignatureContainer(5);
#signatures.delete_loc_files()

recognize_location();
