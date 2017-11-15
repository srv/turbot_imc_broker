// The MIT License (MIT)

// Copyright (c) 2017 Universitat de les Illes Balears

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#include <turbot_imc_broker/turbot_imc_broker.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "turbot_imc_broker");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Read parameters
  TurbotIMCBroker::Params params;
  string outdir, filename;
  nhp.param("outdir", params.outdir, string(""));
  nhp.param("filename", params.filename, string(""));

  TurbotIMCBroker tb;
  tb.setParams(params);
  ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
  spinner.spin();  // spin() will not return until the node has been shutdown
  return 0;
}
