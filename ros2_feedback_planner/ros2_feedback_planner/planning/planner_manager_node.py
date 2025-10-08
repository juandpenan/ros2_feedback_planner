import importlib
import pkgutil


def load_plugin(module_path: str, class_name: str, **kwargs):
    module = importlib.import_module(module_path)
    klass = getattr(module, class_name)
    return klass(**kwargs)


def main():
    planner = load_plugin('ros2_feedback_planner.planning.planner_online', 'OnlinePlanner', api_key='AIzaSyBDH1m_KmYpv3izEJdWFZqfMi6Ldxs7qrw', base_url='https://generativelanguage.googleapis.com/v1beta/openai/')
    text = planner.plan()
    print(text)




if __name__ == '__main__':
    main()
    