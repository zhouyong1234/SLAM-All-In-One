# Umeyama Method
The absolute orientation method of [Umeyama](https://web.stanford.edu/class/cs273/refs/umeyama.pdf) computes the similarity transformation which minimizes the mean squared distance between two point sets *A* and *B* of arbitrary dimension *d*.

The implementation of Umeyama is highly inspired by [Eigen](https://eigen.tuxfamily.org/dox/Umeyama_8h_source.html) and is based on [OpenCV](https://opencv.org/) (2.4.13).

# Reference
S. Umeyama. Least-squares estimation of transformation parameters between two point patterns. Pattern Analysis and Machine Intelligence, IEEE Transactions on, 13(4):376–380, 1991. 