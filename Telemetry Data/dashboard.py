from pathlib import Path
from enum import Enum
import pandas as pd
import streamlit as st
from streamlit.runtime.uploaded_file_manager import UploadedFile


class DataSource(Enum):
    UPLOAD = 0
    DEFAULT = 1
    # This will be implemented later
    SERIAL = 2


BALANCE_BOT_DATA = "balance bot log.csv"


def load_file(path: str):
    bot_data = Path(path)
    if bot_data.exists():
        return bot_data
    else:
        return None


@st.dialog("Upload a Telemetry CSV file")
def upload_file():
    uploaded = st.file_uploader(
        "Upload telemetry file",
        type="csv",
        help="Select a telemetry CSV file to analyse",
    )
    if uploaded is not None:
        st.session_state.telemetry_file = uploaded
    if st.button("Finished"):
        st.session_state.show_upload_dialog = False
        st.rerun()
    return uploaded


@st.cache_data
def load_csv(path: Path | UploadedFile):
    try:
        df = pd.read_csv(path)
        return df
    except (pd.errors.EmptyDataError, pd.errors.ParserError) as e:
        st.error(f"Invalid telemetry data {e}")
        st.stop()


def load_md(filename: str):
    return Path("content", filename).read_text(encoding="utf-8")


def load_data(data_source: DataSource = DataSource.DEFAULT):
    if data_source == DataSource.UPLOAD:
        telemetry_file = upload_file()
    else:
        telemetry_file = load_file(BALANCE_BOT_DATA)

    if telemetry_file is not None:
        telemetry_data = load_csv(telemetry_file)
        st.session_state.datasets[telemetry_file.name] = telemetry_data


def init():
    if "datasets" not in st.session_state:
        st.session_state.datasets = {}
        load_data(DataSource.DEFAULT)


def sidebar_options():
    with st.sidebar:
        if st.button("Upload"):
            st.session_state.show_upload_dialog = True
        if st.session_state.get("show_upload_dialog", False):
            load_data(DataSource.UPLOAD)


def render_title():
    st.title("Balance Bot Telemetry Data")
    st.markdown(load_md("overview.md"))


def main():
    init()
    sidebar_options()
    render_title()


if __name__ == "__main__":
    main()
