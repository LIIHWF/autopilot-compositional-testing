import pandas as pd
import matplotlib.pyplot as plt
from script.run_test_case.common import PathManager
import json
import os
import matplotlib


def round_result(result):
    result = result.copy()
    for key in result:
        if isinstance(result[key], float):
            result[key] = round(result[key], 1)
    return result

def verdict_color(verdict):
    if not isinstance(verdict, str):
        return 'white'
    if 'Running' in verdict:
        return 'yellow'
    elif 'A' in verdict:
        return 'pink'
    elif 'PS' in verdict:
        return 'lightgreen'
    elif 'PU' in verdict:
        return 'tan'
    elif 'CS' in verdict:
        return 'lightblue'
    elif 'CU' in verdict:
        return 'plum'
    elif 'Fsw' in verdict:
        return 'orange'
    elif 'Blk' in verdict:
        return 'lightgray'
    else:
        return 'white'


class Dashboard:
    def __init__(self, autopilot: str, vista: str, ve: float):
        self.autopilot = autopilot
        self.vista = vista
        self.ve = ve
        self.path_manager = PathManager(autopilot, vista, ve)
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(10, 5)
        self.xa_index_map = dict()
        self.xf_index_map = dict()
        self.table = self._init_table()

    def query_result(self, test_case):
        verdict_path = self.path_manager.verdict_path(test_case)
        if not os.path.exists(verdict_path):
            return None
        with open(verdict_path) as f:
            data = json.load(f)
        for key in test_case:
            if key not in data:
                return None
            if abs(test_case[key] - data[key]) > 1e-5:
                return None
        return data

    def _init_table(self):
        results = []
        with open(self.path_manager.test_case_path()) as f:
            test_cases = json.load(f)
        
        critical_test_case = None
        
        for test_case in test_cases:
            if abs(test_case['ve'] - self.ve) < 1e-5:
                if critical_test_case is None:
                    critical_test_case = test_case
                result = self.query_result(test_case)
                if result is not None:
                    results.append(round_result(result))
                else:
                    result = test_case.copy()
                    result['verdict'] = ''
                    results.append(round_result(result))
        df = pd.DataFrame(results)
        if self.vista != 'crossing_with_traffic_lights':
            df = df.pivot(index='xa', columns='xf', values='verdict')
        else:
            df.set_index('xf', inplace=True)
            for col in df.columns:
                if col not in ['verdict', 'xf']:
                    df.drop(col, axis=1, inplace=True)
            df = df.sort_values(by='xf')
            df = df.transpose()

        df.fillna('-', inplace=True)
        if self.vista != 'crossing_with_traffic_lights':
            table = self.ax.table(cellText=df.values, rowLabels=df.index, colLabels=df.columns, loc='center', cellLoc='center',
                                cellColours=[[verdict_color(item) for item in row] for row in df.values])
            table.add_cell(0, -1, width=table[0, 0].get_width(), height=table[0,0].get_height(), text='xa \\ xf')
        else:
            table = self.ax.table(cellText=df.values, rowLabels=['Verdict'], colLabels=df.columns, loc='center', cellLoc='center',
                                cellColours=[[verdict_color(item) for item in row] for row in df.values])
            table.add_cell(0, -1, width=table[0, 0].get_width(), height=table[0,0].get_height(), text='xf')
        for i, xa in enumerate(df.index):
            self.xa_index_map[xa] = i + 1
        for i, xf in enumerate(df.columns):
            self.xf_index_map[xf] = i    
        
        if critical_test_case['xf'] > 0 or ('xa' in critical_test_case and critical_test_case['xa'] > 0):
            critical_xf_cell = table.get_celld()[0, self.xf_index_map[round(critical_test_case['xf'], 1)]]
            critical_xf_cell.get_text().set_color('red')
            critical_xf_cell.get_text().set_fontweight('bold')
            
            if 'xa' in critical_test_case:
                critical_xa_cell = table.get_celld()[self.xa_index_map[round(critical_test_case['xa'], 1)], -1]
                critical_xa_cell.get_text().set_color('red')
                critical_xa_cell.get_text().set_fontweight('bold')
        
        plt.tight_layout()
        return table

    def get_table_cell(self, xf, xa):
        if self.vista == 'crossing_with_traffic_lights':
            xai = 1
        else:
            xai = self.xa_index_map[round(xa, 1)]
        xfi = self.xf_index_map[round(xf, 1)]
        return self.table.get_celld()[xai, xfi]

    def mark_running(self, test_case):
        test_case = test_case.copy()
        if 'xa' not in test_case:
            test_case['xa'] = ''
        cell = self.get_table_cell(test_case['xf'], test_case['xa'])
        cell.get_text().set_text('Running')
        cell.set_color(verdict_color('Running'))
        self.fig.canvas.start_event_loop(1)
        self.fig.canvas.draw_idle()

    def update_result(self, result):
        result = result.copy()
        if 'xa' not in result:
            result['xa'] = ''
        cell = self.get_table_cell(result['xf'], result['xa'])
        cell.get_text().set_text(result['verdict'])
        cell.set(color=verdict_color(result['verdict']))
        cell.set(edgecolor='black')
        self.fig.canvas.start_event_loop(1)
        self.fig.canvas.draw_idle()

    def pause(self, seconds: float):
        plt.ion()
        plt.pause(seconds)
        plt.ioff()

    def show(self, block: bool):
        plt.get_current_fig_manager().set_window_title(f'Testing the {self.autopilot.capitalize()} autopilot for {self.vista} with ve={self.ve}')
        plt.axis('off')
        plt.show(block=block)