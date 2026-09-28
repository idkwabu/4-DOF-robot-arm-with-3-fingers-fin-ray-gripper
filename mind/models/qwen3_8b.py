from __future__ import annotations

import shutil
import subprocess
import sys
import time
import urllib.request

try:
    import ollama
except ImportError:
    print("The Ollama Python package is not installed. Run: pip install ollama", file=sys.stderr)
    raise SystemExit(1) from None


MODEL_NAME = "qwen3:8b"
OLLAMA_URL = "http://localhost:11434"
STARTUP_TIMEOUT_SECONDS = 15.0
REQUEST_TIMEOUT_SECONDS = 300.0
MAX_HISTORY_MESSAGES = 20
EXIT_COMMANDS = frozenset({"byr", "bye", "exit", "quit"})
SYSTEM_PROMPT = "You are a helpful assistant. Answer clearly and concisely."


class ChatbotError(RuntimeError):
    pass


def _configure_output() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def is_ollama_running() -> bool:
    try:
        with urllib.request.urlopen(f"{OLLAMA_URL}/api/tags", timeout=5) as response:
            return response.status == 200
    except Exception:
        return False


def start_ollama() -> None:
    if is_ollama_running():
        return

    ollama_path = shutil.which("ollama")
    if ollama_path is None:
        raise ChatbotError(
            "Ollama is not running and its executable was not found. "
            "Install Ollama, then run this chatbot again."
        )

    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        subprocess.Popen(
            [ollama_path, "serve"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=creation_flags,
        )
    except OSError as exc:
        raise ChatbotError(f"Ollama could not be started: {exc}") from exc

    deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if is_ollama_running():
            return
        time.sleep(0.5)

    raise ChatbotError(
        "Ollama did not become ready in time. Run 'ollama serve' and check the server logs."
    )


def ensure_model_installed(client: ollama.Client) -> None:
    try:
        models = client.list().models
    except Exception as exc:
        raise ChatbotError(f"Could not list Ollama models: {exc}") from exc

    installed_names = {
        model.model
        for model in models
        if isinstance(getattr(model, "model", None), str)
    }
    if MODEL_NAME not in installed_names:
        raise ChatbotError(
            f"Model '{MODEL_NAME}' is not installed. Run: ollama pull {MODEL_NAME}"
        )


def is_exit_command(message: str) -> bool:
    normalized = " ".join(message.casefold().split()).strip(".!?")
    return normalized in EXIT_COMMANDS


def request_reply(
    client: ollama.Client,
    history: list[dict[str, str]],
    user_message: str,
) -> str:
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        *history,
        {"role": "user", "content": user_message},
    ]

    try:
        stream = client.chat(
            model=MODEL_NAME,
            messages=messages,
            stream=True,
            think=False,
            keep_alive="10m",
            options={"temperature": 0.7},
        )
        parts: list[str] = []
        print("Qwen: ", end="", flush=True)
        for chunk in stream:
            content = getattr(getattr(chunk, "message", None), "content", None)
            if isinstance(content, str) and content:
                parts.append(content)
                print(content, end="", flush=True)
    except KeyboardInterrupt:
        print("\nGeneration cancelled.")
        raise
    except Exception as exc:
        print()
        raise ChatbotError(f"Ollama request failed: {exc}") from exc

    answer = "".join(parts).strip()
    if not answer:
        print()
        raise ChatbotError("The model returned an empty response. Please try again.")

    print()
    history.extend(
        [
            {"role": "user", "content": user_message},
            {"role": "assistant", "content": answer},
        ]
    )
    if len(history) > MAX_HISTORY_MESSAGES:
        del history[:-MAX_HISTORY_MESSAGES]
    return answer


def run_chatbot(control_client: ollama.Client, chat_client: ollama.Client) -> int:
    print("Checking Ollama server...")
    try:
        start_ollama()
        ensure_model_installed(control_client)
    except ChatbotError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Chatbot ready. Model: {MODEL_NAME}")
    print("Type 'exit', 'quit', or 'bye' to stop.")
    history: list[dict[str, str]] = []

    while True:
        try:
            user_message = input("\nYou: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nGoodbye!")
            return 0

        if not user_message:
            continue
        if is_exit_command(user_message):
            print("Qwen: Goodbye!")
            return 0

        try:
            request_reply(chat_client, history, user_message)
        except KeyboardInterrupt:
            continue
        except ChatbotError as exc:
            print(f"Error: {exc}", file=sys.stderr)
        except Exception as exc:
            print(f"Unexpected error: {exc}", file=sys.stderr)


def main() -> int:
    _configure_output()
    try:
        control_client = ollama.Client(host=OLLAMA_URL, timeout=10.0)
        chat_client = ollama.Client(host=OLLAMA_URL, timeout=REQUEST_TIMEOUT_SECONDS)
        return run_chatbot(control_client, chat_client)
    except KeyboardInterrupt:
        print("\nGoodbye!")
        return 0
    except Exception as exc:
        print(f"Unexpected startup error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
