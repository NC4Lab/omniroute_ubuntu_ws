    def checkStatus(current_status_enum, new_status_enum):
        """ Checks if the new state is valid and updates the state if it is """

        if current_status_enum == MazePlot.Status.EXCLUDED:
            return False
        elif current_status_enum == MazePlot.Status.ERROR:
            return False
        elif current_status_enum == MazePlot.Status.DISABLED:
            if new_status_enum != MazePlot.Status.DISABLED and \
                new_status_enum != MazePlot.Status.ENABLED and \
                new_status_enum != MazePlot.Status.ERROR and \
                new_status_enum != MazePlot.Status.WARNING:
                return False
        return True