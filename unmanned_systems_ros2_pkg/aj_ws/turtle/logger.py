#!/usr/bin/env python3

import datetime
import csv


class Logger:
    def __init__(self, file_path, log_header):
        self.file_path = file_path
        self.log_header = log_header

    def save_csv(self, log_data):
        fileName = (
            self.file_path + datetime.datetime.now().strftime("%b_%d_%H_%M") + ".csv"
        )

        myFile = open(fileName, "a")
        with myFile:
            writer = csv.writer(myFile)
            writer.writerow(log_data)
