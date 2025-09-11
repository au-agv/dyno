import json

from logging import getLogger
from pathlib import Path

logger = getLogger(__name__)


class JSONWithCommentsDecoder(json.JSONDecoder):

    def __init__(self, **kw):
        super().__init__(**kw)

    def decode(self, s: str):
        s = "\n".join(l for l in s.split("\n") if not l.lstrip(" ").startswith("//"))
        return super().decode(s)


class Cleaner:

    _DAKOTA_FILES_SUFFIXES = [".out", ".rst", ".dkt"]

    def __init__(self, workspace: Path):

        self._workspace: Path = workspace

    @staticmethod
    def rmdir(self, path: Path):
        for entry in path.iterdir():
            if entry.is_dir():
                self._rmdir(entry)
            else:
                entry.unlink()
        path.rmdir()

    def clean(self):

        workspace_folder: Path = Path.cwd()

        logger.info(
            "Looking for files in the DAKOTA case directory: %s", workspace_folder
        )
        listed_files = workspace_folder.glob("./*")

        marked_files = [
            file
            for file in listed_files
            if file.is_file()
            and any(
                [
                    str(file.suffixes).find(suffix) != -1
                    for suffix in self._DAKOTA_FILES_SUFFIXES
                ]
            )
        ]

        marked_directories = [
            run_folder
            for run_folder in workspace_folder.glob("./*")
            if run_folder.is_dir() and run_folder.name.startswith("run")
        ]

        if not list(marked_directories) and not list(marked_files):
            logger.info("No files or directories to clean up.")

        # List the files and directories to be removed with a warning.
        if list(marked_files):
            logger.warning(
                "Listing files to be removed: %s",
                [file.name for file in list(marked_files)],
            )

        if list(marked_directories):
            logger.warning(
                "Listing directories to be removed: %s",
                [directory.name for directory in list(marked_directories)],
            )

        # Confirm the user input.
        confirmation_prompt = "Confirm [Y/n]: "
        confirmation = input(confirmation_prompt)
        while confirmation != "Y" and confirmation != "n":
            logger.warning("Unknown input.")
            confirmation = input(confirmation_prompt)

        # Remove the files
        if confirmation == "Y":
            for file in marked_files:
                file.unlink()
            for directory in marked_directories:
                self._rmdir(directory)
        else:
            logger.info("Aborting ...")
