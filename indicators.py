from enum import Enum
import numpy as np

class REGULATION_TYPE(Enum):
    INCREASE = 1
    REDUCE = 2
    NOACT = 3

class WorkloadStatistics:
    def __init__(self, num_dimensions):
        self.num_dimensions = num_dimensions
        self.n = 0
        self.means = [0.0] * num_dimensions
        self.m2s = [0.0] * num_dimensions

    def update(self, sample):
        self.n += 1
        for i in range(self.num_dimensions):
            delta = sample[i] - self.means[i]
            self.means[i] += delta / self.n
            delta2 = sample[i] - self.means[i]
            self.m2s[i] += delta * delta2

        # Correcting bias in estimated variance (Welford's algorithm)
        if self.n > 1:
            self.variances = [m2 / self.n for m2 in self.m2s]
        else:
            self.variances = [0.0] * self.num_dimensions  # Avoid division by zero

    def is_low_workload(self, new_sample, alpha_threshold=1)->list:
        """
        Check if the new sample is within 'threshold' standard deviations from the mean.
        """
        if self.n < 2:  # Need at least two samples to have a meaningful variance
            return [0] * self.num_dimensions
        std_devs = [np.sqrt(var) for var in self.variances]
        return [1 if a <= b - alpha_threshold * c else 0 for a, b, c in zip(new_sample, self.means, std_devs)]

class Indicators:
    def __init__(
        self, 
        Nd: list[int], 
        Np: list[int], 
        Ns: list[int], 
        Nb: list[int], 
        Delta_t: int,
        mdi_theta: float,
        isri_lambda: float,
        f_s: list[float],
        pw_time: list[float],
        pmdi_w: int
    ):
        self.Nd = Nd.copy()
        self.Np = Np.copy()
        self.Ns = Ns.copy()
        self.Nb = Nb.copy()
        self.f_s = f_s.copy()
        self.workload_statistics = WorkloadStatistics(len(pw_time))
        self.workload_statistics.update(pw_time)
        self.pw_time = pw_time.copy()
        
        self.pre_Nd = [0] * len(self.Nd)
        self.pre_Np = [0] * len(self.Np)
        self.pre_Nb = [0] * len(self.Nb)
        self.pre_pre_Nb = [0] * len(self.Nb)

        self.Delta_t = Delta_t
        self.mdi_theta = mdi_theta
        self.isri_lambda = isri_lambda
        self.regulation_type = REGULATION_TYPE.NOACT

        self.mdi_history = []
        self.pmdi_w = pmdi_w

    def update(
        self, 
        Nd: list[int], 
        Np: list[int], 
        Ns: list[int], 
        Nb: list[int], 
        f_s: list[float],
        pw_time: list[float]
    )->None:
        self.pre_Nd = self.Nd.copy()
        self.pre_Np = self.Np.copy()
        self.pre_pre_Nb = self.pre_Nb.copy()
        self.pre_Nb = self.Nb.copy()

        self.Nd = Nd.copy()
        self.Np = Np.copy()
        self.Ns = Ns.copy()
        self.Nb = Nb.copy()
        self.f_s = f_s.copy()
        self.pw_time = pw_time.copy()
        self.workload_statistics.update(pw_time)
        self.regulation_type = REGULATION_TYPE.NOACT

    # TODO: the number of items in different list variables (done)
    def check(self)->tuple[REGULATION_TYPE, list[int]]:
        ##### Check if we need to regulate timer frequency
        regulation_position = []
        if any(item > 0 for item in self.Nd):
            ##### 1. Message Drop Indicator (MDI)
            # mdr1 = Nd / (Np + pre_Nb)
            # mdr2 = Nd / (Nd + Nb + Ns)
            mdi1 = [a / (b + c) for a, b, c in zip(self.Nd, self.Np, self.pre_Nb)]
            mdi2 = [a / (a + b + c) for a, b, c in zip(self.Nd, self.Nb, self.Ns)]
            print("@@@ Immediate Message Drop Indicator: ", mdi1)
            # print("Message Drop Rate 2: ", mdi2)
            self.mdi_history.append(mdi1)

            regulation_position = [1 if item >= self.mdi_theta else 0 for item in mdi1]
            if any(item == 1 for item in regulation_position):
                self.regulation_type = REGULATION_TYPE.REDUCE
                return self.regulation_type, regulation_position
            
            regulation_position = [1 if item >= self.mdi_theta else 0 for item in mdi2]
            if any(item == 1 for item in regulation_position):
                self.regulation_type = REGULATION_TYPE.REDUCE
                return self.regulation_type, regulation_position

            ##### 2. Persistent Message Drop Indicator (PMDI)
            # mdg = (self.Nd - self.pre_Nd) / (self.Np + self.pre_Nb - self.pre_Np - self.pre_pre_Nb)
            # mdg = [abs((a - b) / (c + d - e - f)) if c + d - e - f != 0 else 0 for a, b, c, d, e, f in zip(self.Nd, self.pre_Nd, self.Np, self.pre_Nb, self.pre_Np, self.pre_pre_Nb)]
            # print("### Message Drop Gradient: ", mdg)

            #TODO: change it to check several periods for each dimension (done)
            # len_of_mdi_history = len(self.mdi_history)
            # mdis = []
            # for idx in range(self.pmdi_w):
            #     mdis.append(self.mdi_history[len_of_mdi_history - idx - 1])
            
            # pmdi = [1 if all(0 < elem < self.mdi_theta for elem in elems) else 0 for elems in zip(*mdis)]
            # # pmdi = [1 if 0 < item < self.mdi_theta else 0 for item in (mdi1)]
            # print("### Persistent Message Drop Indicator: ", pmdi)
            
            # regulation_position = [1 if item == 1 else 0 for item in pmdi]
            # if any(item == 1 for item in regulation_position):
            #     self.regulation_type = REGULATION_TYPE.REDUCE
            #     return self.regulation_type, regulation_position
        else:
            ##### 3. Increasing Sampling Rate Indicator (ISRI)
            new_f_s = [self.isri_lambda * item for item in self.f_s]
            condition_result_1 = [1 if a > b else 0 for a, b in zip(new_f_s,self.pw_time)]
            condition_result_2 = self.workload_statistics.is_low_workload(self.pw_time)
            for idx in range(len(condition_result_1)):
                if condition_result_1[idx] == 1 and condition_result_2[idx] == 1:
                    regulation_position.append(1)
                else:
                    regulation_position.append(0)
            self.regulation_type = REGULATION_TYPE.INCREASE
            return self.regulation_type, regulation_position

        return self.regulation_type, regulation_position

        