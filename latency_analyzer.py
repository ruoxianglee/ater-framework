import numpy as np
import pandas as pd

from caret_analyze.runtime.path import Path

def node_latency_statistics(
    path: Path,
    lstrip_s=0,
    rstrip_s=0,
) -> tuple[list, list]:
    def calc_latency_from_path_df(target_columns: list[str]) -> np.ndarray:
        target_df = path.to_dataframe(
            remove_dropped=remove_dropped,
            treat_drop_as_delay=treat_drop_as_delay,
            lstrip_s=lstrip_s,
            rstrip_s=rstrip_s,
        )[target_columns]
        source_stamps_ns = np.array(target_df.iloc[:, 0].values)
        dest_stamps_ns = np.array(target_df.iloc[:, -1].values)
        latency_ns = dest_stamps_ns - source_stamps_ns
        if remove_dropped:
            latency_ns = latency_ns.astype('int64')
        return latency_ns

    remove_dropped = False
    treat_drop_as_delay=False

    node_latency_list = []
    pubsub_latency_list = []

    for i, node_path in enumerate(path.node_paths):
        if i == 0 and path.include_first_callback:
            first_cb_columns = path.column_names[0:2]
            latency = calc_latency_from_path_df(first_cb_columns)
            
            latency = latency[[not pd.isnull(_) for _ in latency]]
            node_latency_list.append(np.max(latency * 1.0e-6))

        elif i == len(path.node_paths)-1 and path.include_last_callback:
            last_cb_columns = path.column_names[-2:]
            latency = calc_latency_from_path_df(last_cb_columns)

            latency = latency[[not pd.isnull(_) for _ in latency]]
            node_latency_list.append(np.max(latency * 1.0e-6))

        elif node_path.column_names != []:
            _, latency = node_path.to_timeseries(
                remove_dropped=remove_dropped,
                treat_drop_as_delay=treat_drop_as_delay,
                lstrip_s=lstrip_s,
                rstrip_s=rstrip_s,
            )
            
            latency = latency[[not pd.isnull(_) for _ in latency]]
            node_latency_list.append(np.max(latency * 1.0e-6))

    for comm_path in path.communications:
        _, pubsub_latency = comm_path.to_timeseries(
            remove_dropped=remove_dropped,
            treat_drop_as_delay=treat_drop_as_delay,
            lstrip_s=lstrip_s,
            rstrip_s=rstrip_s,
        )
        pubsub_latency = pubsub_latency[[not pd.isnull(_) for _ in pubsub_latency]]
        pubsub_latency_list.append(np.max(pubsub_latency * 1.0e-6))

    return node_latency_list, pubsub_latency_list

def latency_statistics(
    path: Path,
    lstrip_s=0,
    rstrip_s=0,
    analyze_node_latency=False
) -> tuple[list, list, float]:
    remove_dropped = False
    treat_drop_as_delay=False
    node_latency_list=[]
    pubsub_latency_list=[]

    if analyze_node_latency:
        node_latency_list, pubsub_latency_list = node_latency_statistics(path, lstrip_s, rstrip_s)

    _, e2e_latency = path.to_timeseries(
        remove_dropped=remove_dropped,
        treat_drop_as_delay=treat_drop_as_delay,
        lstrip_s=lstrip_s,
        rstrip_s=rstrip_s,
    )

    e2e_latency = e2e_latency[[not pd.isnull(_) for _ in e2e_latency]]
    return node_latency_list, pubsub_latency_list, np.max(e2e_latency * 1.0e-6)