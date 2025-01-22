import streamlit as st
import pandas as pd
import argparse
import os
import json
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib
import time
import math

parser = argparse.ArgumentParser('Show test results')
parser.add_argument('autopilot', type=str, help='autopilot to test', choices=['apollo', 'autoware', 'carla', 'lgsvl'])
parser.add_argument('vista_type', type=str, help='type of vista', choices=['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights'])
parser.add_argument('ve', type=float, help='speed of the ego vehicle')


class PathManager:
    def __init__(self, autopilot, vista, ve):
        self.autopilot = autopilot
        self.vista = vista
        self.ve = ve
        
    def verdict_dir(self):
        save_dir = f'test_result/verdict/{self.autopilot}/{self.vista}/ve={round(self.ve, 1)}'
        return save_dir

    def verdict_path(self, test_case):
        if self.vista == 'crossing_with_traffic_lights':
            file_name = f'xf={round(test_case["xf"], 1)}.json'
        else:
            file_name = f'xf={round(test_case["xf"], 1)};xa={round(test_case["xa"], 1)}.json'
        verdict = os.path.join(self.verdict_dir(), file_name)
        return verdict
    
    def test_case_path(self):
        path = f'test_case/{self.autopilot}_{self.vista}.json'
        return path


# set wide screen
st.set_page_config(
    page_title="CCTest",
    page_icon="👋",
    layout="wide",
)

def round_result(result):
    result = result.copy()
    for key in result:
        if isinstance(result[key], float):
            result[key] = round(result[key], 1)
    return result


ISSUE_MAP = {
    'BOTH_IN_JUNCTION': 'p1',
    'EGO_STOPPED': 'p2',
    'RUN_RED_LIGHT': 'p3',
    'IN_JUNCTION_WHEN_SIDE_GREEN': 'p4'
}

def simple_verdict(result):
    if result['verdict'] == 'collision':
        verdict = 'A'
    elif result['verdict'] == 'progress':
        verdict = 'P'
    elif result['verdict'] == 'caution':
        verdict = 'C'
    elif result['verdict'] == 'planning_failed':
        verdict = 'Fsw'
    elif result['verdict'] == 'blocking':
        verdict = 'Blk'
    else:
        raise ValueError(f'Unknown verdict: {result["verdict"]}')
    if verdict == 'A':
        if 'ego_fault' in result['safety_issues']:
            verdict += 'e'
        if 'arriving_fault' in result['safety_issues']:
            verdict += 'a'
    elif verdict in ['P', 'C']:
        if len(result['safety_issues']) == 0:
            verdict += 'S'
        else:
            verdict += 'U'
            for issue in ISSUE_MAP:
                if issue in result['safety_issues']:
                    verdict += ISSUE_MAP[issue]
    return verdict


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
        self.table, self.df = self._init_table()

    def query_result(self, test_case):
        verdict_path = self.path_manager.verdict_path(test_case)
        if not os.path.exists(verdict_path):
            return None
        with open(verdict_path) as f:
            data = json.load(f)
        for key in test_case:
            if key not in data:
                return None
            if abs(test_case[key] - data[key]) > 1e-1:
                return None
        data['verdict'] = simple_verdict(data)
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
                    result['verdict'] = 'pending'
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
        return table, df

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

args = parser.parse_args()
dashboard = Dashboard(args.autopilot, args.vista_type, args.ve)

# use st to show the dashboard.df, show the hole table without the need to scroll

def color_cells(val):
    """
    根据单元格内容返回样式
    """

    return f"background-color: {verdict_color(val)}; color: black;"

# 使用 Pandas Styler

styled_df = dashboard.df.style.applymap(color_cells)
st.dataframe(styled_df, height=800)

# def render_button_table(df):
#     # 开始 HTML 表格
#     table_html = '<table style="border-collapse: collapse; width: 100%;">'
    
#     # 添加表头
#     table_html += '<thead><tr>'
#     table_html += '<th style="border: 1px solid black; padding: 5px; text-align: center;">xa\\xf</th>'  # 行索引标题
#     for col in df.columns:
#         table_html += f'<th style="border: 1px solid black; padding: 5px; text-align: center;">{col}</th>'
#     table_html += '</tr></thead>'
    
#     # 添加表格内容
#     table_html += '<tbody>'
#     for idx, row in df.iterrows():
#         table_html += '<tr>'
#         # 添加行索引按钮, make text bold
#         table_html += f"""
#         <td style="border: 1px solid black; padding: 0px; text-align: center;">
#             <div style="width: 100%; padding: 5px; margin: 0; color: black; cursor: pointer; font-weight: bold;">
#                 {idx}
#             </div>
#         </td>
#         """
#         # 添加普通单元格按钮
#         for val in row:
#             table_html += f"""
#             <td style="border: 1px solid black; padding: 5px; text-align: center;">
#                 <button onclick="alert('You clicked {val}')"
#                         style="width:100%;padding:5px;margin:0;border:1px solid #ddd;background-color:{verdict_color(val)};color:black;cursor:pointer;">
#                     {val}
#                 </button>
#             </td>
#             """
#         table_html += '</tr>'
#     table_html += '</tbody></table>'
    
#     return table_html

# def create_table_with_buttons(df):
#     # 构建表头
#     cols = st.columns(len(df.columns) + 1)  # 列数 = 数据列数 + 行索引
    
#     st.markdown("""
#     <style>
#     [data-testid=column]:nth-of-type(1) [data-testid=stVerticalBlock]{
#         gap: 0rem;
#     }
#     </style>
#     """,unsafe_allow_html=True)
    
#     cols[0].markdown("**Index**")  # 行索引标题
#     for i, col_name in enumerate(df.columns):
#         cols[i + 1].markdown(f"**{col_name}**")  # 数据列标题

#     # 构建表格行
#     for idx, row in df.iterrows():
#         cols = st.columns(len(row) + 1, gap='small')  # 每行的列数
#         # 添加行索引按钮
#         if cols[0].button(label=str(idx)):
#             st.write(f"You clicked on Index: {idx}")
#         # 添加数据按钮
#         for i, val in enumerate(row):
#             if cols[i + 1].button(label=str(val),key=f'{idx}_{i}'):
#                 st.write(f"You clicked on {val}")

# custom_css = """
# <style>
# /* 强制覆盖 .st-emotion-cache-* 的 gap */
# [class^="st-emotion-cache"] {
#     gap: 0px !important; /* 全局行列间距设置为 0 */
# }

# /* 如果特定类仍未被覆盖，可以直接指定类名 */
# .st-emotion-cache-1rrhfa {
#     gap: 0px !important; /* 针对特定类的覆盖 */
# }
# </style>
# """

# # 应用自定义 CSS
# create_table_with_buttons(dashboard.df)
# st.markdown(custom_css, unsafe_allow_html=True)
