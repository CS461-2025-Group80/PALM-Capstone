<!-- YYYY-MM-DD-[TeamID]-TeamCharter.md -->

# Team Charter & Working Agreement

# **Team Members & Roles**

| Member Name | Member Role(s) | Primary Responsibilities | Backups | Contact Info |
| :---- | :---- | :---- | :---- | :---- |
| Justice Peyton | ROS2 (robot operating system 2\) Developer, Communication & Collaboration Lead | \- Training and familiarizing with ROS2 \- Teaching and providing resources regarding ROS2 to team members \- Integrating ROS2 with other team members’ work \- Ensuring solid communication between the team and our project proposer \- Ensuring that the team can properly collaborate over Discord, Google Docs, and GitHub \- Ensuring timely collaboration on Capstone assignments | \- Communication backup: Luke | peytonju@oregonstate.edu, 650-750-4920 |
| Luke Hashbarger | VR/AR, Note taker  | \- Expand VR/AR robot operation \- Take notes during in person meetings \- Ensure remote meeting transcripts and their summaries are  | \- Note taker backup: Justice | hashbarl@oregonstate.edu |
| Andrew Fief | Storage/Vision Developer, Document Editor | \- Develop a robot \-\> azure cloud storage pipeline for large amounts of image data \- Effectively label and provide metadata with images to set up computer vision success \- Capstone document and technical paper finalizations including formatting, grammar checks, rubric requirement verification | \- Document Finalization Backup: Luke  | fiefa@oregonstate.edu 541-223-2925 |

# **Decision-Making Model**

Decisions will be based on a majority-rule of the team. However, if there has been a distinctive lack of activity (no comments or actions from the team for at least 2 days, other than for explicit course breaks), the one active student is free to make *reasonable* decisions as they please in order to maintain project momentum.

Decisions that involve hardware or software directly affecting REVOBOTS’ other work requires consultation with the project proposer and REVOBOTS staff. Everyone is expected to sensibly and reasonably involve REVOBOTS in any matter in general.

If disagreement is present, a neutral party (another team member or a member of REVOBOTS staff) must break it.

# **Meeting Cadence & Tools**

| Meeting | Recurrence | Duration | Required Tools | Purpose | Attendance Expectation |
| :---- | :---- | :---- | :---- | :---- | :---- |
| Capstone & REVOBOTS Staff Meeting | Mondays at 7:00 PM on Microsoft Teams | 30 minutes to 2 hours (formally 1 hour) | The required tools for these meetings are exclusively based on recent/relevant work to the meeting (which may include certain repositories, CI output, hardware or software access, etc). | \- Capstone team stand-ups \- REVOBOTS updates and feedback \- New task assignments for the Capstone team | Every team member is expected to attend these meetings. |
| Capstone & TA Stand-up | Fridays at 1:30 PM on Zoom | 5 to 20 minutes (formally 10 minutes) | This will almost always require Canvas access. Other sensible tools may also be used if requested by the TA (repositories, documentation, etc). | \- Capstone assignment clarification \- Capstone course comments and updates \- TA feedback | Every team member is expected to attend these meetings. |

# **Risk Management & Escalation Path**

## Time Waste Risk

### Early Warning Indicators

REVOBOTS has had a very slow roll-out of our access to critical REVOBOTS resources, such as repository and hardware access, on top of providing us with a very ambiguous description of the project overall.

These occurrences will cause us to waste time if we don’t continue to press REVOBOTS for access and explanations. We have already begun to mitigate this by multitasking– occupying ourselves with something important while we ask for access to something we’ll need in the future. We have, so far, been spending this time doing research and training in preparation for getting hardware and software access, which we believe, as the project proposer does, will help us start fast when we begin real development.

### Escalation

We will escalate this concern of REVOBOTS being slow directly with our project proposer through Discord, which they are very responsive on. This is not something that we need to provide evidence on, as REVOBOTS is well aware of this problem.

We have established very immediate and direct communication with REVOBOTS staff and thus have a meaningful way to communicate our concerns regarding this. We have also already begun talking about this to our project proposer and met on 10/28 to discuss this in-person.

## Team Member Participation Risk

### Early Warning Indicators

A slow-down of team member participation and project ambiguity so far has led to team silence and a lack of participation.

### Escalation

Every team member, as their role as a student and as an adult, is responsible for being timely and communicating what they are able or not able to do for the Capstone course and project. If, for whatever reason, a team member is not able to uphold this, they will be directly messaged and asked to allot time for the project. If this is not met, the project Capstone instructor will be contacted.

The team’s communication method, Discord, is available for our project proposer to see. The project proposer may, at any point, interact with the team and request availability as well as contact the Capstone instructor in the case that they do not feel that team members are participating enough.

# **Conflict Resolution & Accountability**

## Triggers

These are a list of “triggers” for conflict.

1. A team member missing a single meeting without notice or otherwise acknowledgement  
2. A team member contributes/responds on the same date as a deadline  
3. A team member fails to respond to a message within 36 hours

## Steps

These are the steps to be taken after a trigger occurs.

1. Within 1 day of a trigger, the team lead (Justice) or the backup (Luke) will check-in with the violating team member to identify barriers, restrictions, or other difficulties.  
2. Within 5 days of a trigger and no form of acknowledgement, the team lead or the backup will notify the project mentor and Capstone instructor and request action from the Capstone instructor.

## Accountability

Every team member is expected to communicate. You can avoid triggering any of the aforementioned triggers by simply communicating.

- You should never be afraid to express that you can/can’t do something.  
- You should always notify the team if you need more time on something.

That being said, every team member is expected to be able to back themselves up if they are requested to prove what they have worked on (to prevent people from simply talking their way through the project). There are a lot of ways you can prove this.

- GitHub commits (including pull requests)  
- Document edit history  
- Meeting notes  
- Emails  
- REVOBOTS collaboration

# **Definition of Done (DoD) & Quality Gates**

Note: our project proposer has explicitly stated to us that we are not strictly bound by what the Capstone project’s posting states on the OSU EECS page. They have taken a very organic approach with our team and has stated that for right now, we only have two major tasks. In other words, our team has an evolving “definition of done.” Upon completing our current two major tasks, which our project proposer expects us to complete by the end of Fall term, we will have additional tasks given to us, therefore changing our end-goals.

The project proposer wanted to accommodate our skills and lack of experience with robotics and AI– they have stated to us that they want us to learn above anything else.

| Task | Task Description | Tests | Code Review Process | Security Requirements | Documentation Updates | CI Jobs |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| | | | | | | |
| Remote Controller Support | Two instances of ROS2 must wirelessly communicate over IP. One of these instances, which can be on any machine, takes controller input. This instance then sends another instance of ROS2, perhaps not on the same machine, that controller input. The desired effect is that we can use a controller to control a REVOBOTS robot connected to the internet. | The following must be satisfied. 1\. Regardless of distance, an internet connection is the absolute minimum for remotely moving a robot. 2\. The input should have a latency less than 2 seconds. | REVOBOTS staff will provide a peer review of the code created for this. It must follow the standard that REVOBOTS uses for its code and must be easily readable. The expectation is that we need to produce a minimum-viable method of controlling the robot. This, per our project proposer’s instruction, can be done how we wish. | The IP communication MUST be encrypted (HTTP can NOT be used\!). | New documentation will need to be created explicitly for this, as this is a brand new feature. This must follow the REVOBOTS documentation standard. | None. This is tested with an actual robot and is not possible to virtualize with the current REVOBOTS environment. |

# **Accessibility & Inclusion Practices**

Specify meeting norms (time zones, turn-taking, note-taking), accommodation process, and how barriers will be surfaced/addressed.

The meeting will be conducted in PST. Note taking in online meetings will be done with built in transcribing tools and AI Chatbots to create a summary. In person note taking will be conducted by Luke or in their absence Justice. These notes and summaries will be posted to the teams discord. Members not able to make meetings will be able and expected to read the meeting summaries and contribute meaningfully in other ways (such as putting more work into future/current assignments).

Every team member is expected to speak up and ensure that their needs are met. Any discrimination, silencing, and otherwise non-inclusive events are prohibited and should be immediately commented on to the team and, if needed, to our Capstone instructor, Kirsten Winters.

# **Policy Owners & Review Dates**

Assign an owner and next review date for each section; explain how updates are proposed and approved.

| Policy | Owner | Review Date |
| :---- | :---- | :---- |
| Team Members & Roles | Andrew | 10/1/2025 |
| Decision-Making Model | Luke | 10/1/2025 |
| Meeting Cadence & Tools | Justice | 10/1/2025 |
| Risk Management & Escalation Path | Andrew | 10/1/2025 |
| Conflict Resolution & Accountability | Justice | 10/1/2025 |
| Definition of Done (DoD) & Quality Gates | Luke | 10/1/2025 |
| Accessibility & Inclusion Practices | Andrew | 10/1/2025 |

A meeting will be conducted the week of December 1st in order to review all policies and ensure they are optimal for team operations. Updates will be proposed during the meeting, discussed, and unanimously agreed to. If an agreement cannot be reached, the previous policy will remain in place and the issue will be discussed with our project partner or TA. If a member wants to update a policy sooner, they shall message the group in the shared discord and schedule a meeting within the week to discuss the change. 

# **Links & Artifacts**

## Links

### CI

The CI config file is available [here](https://github.com/Aadi0032007/PALM/blob/main/.github/workflows/ci.yml) with the linter config [here](https://github.com/Aadi0032007/PALM/blob/main/.flake8).

The logs of a successfully ran CI is [here](https://github.com/Aadi0032007/PALM/actions/runs/18984432985/job/54224747971) on GitHub. 

### Tests

Available [here](https://github.com/Aadi0032007/PALM/tree/main/tests).

Tests live in root/tests and we will run them with pytest. We don’t expect the legacy code we inherited to pass all of our new configs, and so until it is updated, our CI config file uses echo commands in place of real tests being run. All new capstone code will use pytest.

### Contributing File

The CONTRIBUTING.md file is available [here](https://github.com/Aadi0032007/PALM/blob/main/CONTRIBUTING.md) on the official REVOBOTS repository.

## Artifacts Note

We do not have any explicit software artifacts at this time. We have recently obtained physical access to a robot that we will soon begin to work with and have also gained keycard access to the lab it is stored at in Kelley Engineering Center.

### Meeting Transcripts

Lots of the information on this was based on our initial team meeting on 10/13/2025. This is attached as “transcript.docx”.

# **Assignment Contributions**

All contributions are visible in the assignment’s edit history, [here](https://github.com/CS461-2025-Group80/PALM-Capstone/commits/main/documents/team-charter-and-working-agreement/2025-11-02-80-TeamCharter.md).

The contributions to the CONTRIBUTING.md file are [here](https://github.com/Aadi0032007/PALM/commits/main/CONTRIBUTING.md).

Please note that team members contributed to a Google Document. We eventually moved things over to GitHub. This means that the contributions are not entirely accurate.