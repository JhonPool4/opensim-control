import matplotlib.pyplot as plt

class Plotter():
    def __init__(self, width=4, height=4, nrows=3, ncols=2, max_simtime=4, headers=None, labels=[]):
        # configuration of figures
        font = {'family':'serif',
                'weight' : 'normal',
                'style': 'normal',
                'size'   : '10'}
        lines={'linewidth': '2',
                'linestyle': '-'}                
        axes = {'labelsize': 'medium',
                'titlesize': 'medium',
                'linewidth': '1',
                'grid': 'True',
                'facecolor': 'white',
                'edgecolor': 'k'}
        # pass in the dict as kwargs   
        plt.rc('font', **font)       
        plt.rc('lines', **lines)
        plt.rc('axes', **axes)                        
        #min_y = -0.25
        #max_y = 1.25
        self.min_x = 0
        self.max_x = max_simtime
        self.headers = headers
        self.labels = labels
        self.nrows = nrows
        self.ncols = ncols
        #plt.ion() 
        self.fig, self.ax = plt.subplots(nrows=nrows, ncols=ncols,figsize=(width, height))
        self.fig.tight_layout()

        
        # useful vectors: time, muscle's activation and line 
        self.time_buf = []     
        self.data_buf = dict(zip(self.headers, [[],[],[],[]]))
        self.line_buf = {}      

 
   
        
        
    def update_figure(self):
        for i in range(self.nrows):
            for j in range(self.ncols):
                name = self.headers[i*self.ncols + j]
                label = self.labels[i*self.ncols + j]
                self.ax[i,j].set_xlabel('time (s)')
                self.ax[i,j].set_ylabel(label)
                self.ax[i,j].set_title(name)
                self.ax[i,j].set_xlim((self.min_x,self.max_x))
                #self.ax[i,j].set_ylim((min_y,max_y))    

                self.ax[i,j].plot(self.time_buf, self.data_buf[name], 'r-')    
        
        plt.show()

    def add_data (self, time, data):
        self.time_buf.append(time)
        for idx, name in enumerate(self.headers):
            self.data_buf[name].append(data[idx]) 

    def reset(self):
        self.time_buf = []     
        self.data_buf = dict(zip(self.headers, [[],[],[],[]]))    
