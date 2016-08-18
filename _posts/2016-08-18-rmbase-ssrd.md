---
layout: post
title: MBASE 기반 SSRD
category: UML
tag: [uml]
---

SSRD는 System and Software Requirements Definition를 말합니다.
시스템과 소프트웨어의 요구사항 정의를 한 문서이며, 
템플릿은 [여기](https://www.google.co.kr/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=0ahUKEwi6rOH-58rOAhWDp5QKHWDYAZoQFgggMAA&url=http%3A%2F%2Fsunset.usc.edu%2Fresearch%2FMBASE%2FEPG%2Ftemplates%2Fversion2.3%2FSSRD_SODA_2_3_2_v1.doc&usg=AFQjCNHYu64ibbPAtqqAXpc7gyTLO5LfKQ&bvm=bv.129759880,d.dGo)에서 받을 수 있습니다.

그리고 MBASE는 
Measurable, Archievable, Relevant, Specific를 의미합니다.
여기에 Timebound가 추가되면 SMART라고도 부른다고 하네요.

<br>

## 가이드라인

이 문서의 가이드라인은 [여기](https://www.google.co.kr/url?sa=t&rct=j&q=&esrc=s&source=web&cd=3&ved=0ahUKEwi6rOH-58rOAhWDp5QKHWDYAZoQFggvMAI&url=http%3A%2F%2Fsunset.usc.edu%2Fresearch%2FMBASE%2Fmbase_team%2FLeanMBASE%2FLeanMBASE_Guidelines_V1.0.doc&usg=AFQjCNHAan9gMIQ5evWpqdgZVPwhfPP2Qw&bvm=bv.129759880,d.dGo)에서 받을 수 있습니다.

가이드라인의 내용을 대충 살펴보면

<br>

### Project Requirements

* PR-1: Delivery in 24 weeks: The system shall be delivered and running on the customer organization’s servers within 24 weeks of the start of the project (This is a “schedule” requirement)
* PR-2: Limited COTS budget: The total of all (annual) costs of licenses for COTS products incorporated into the system shall not exceed $50,000. (This is a “budget” requirement)
* PR-3: Design using UML: The system’s design documents shall use UML as the main modeling notation. (This is a “development” requirement; it was, very likely, imposed because the customer organization’s maintenance staff is trained in UML.)
* PR-4: Browser compatibility: The system shall run on all versions of Internet Explorer, v5 and higher, and on all versions of Netscape, v4.3 and higher. (This is a “deployment” requirement)
* PR-5: Operator training: The developers shall provide ten hours of hands-on operator training in the use of the system to 20 employees of the client organization, such training to start the day after initial deployment testing has been completed. (This is a “transition” requirement)

Project Requirement | << (i) A unique identifier of the project requirement of the form “PR-n,” where n is an integer; (ii) A short title of the Project requirement e.g., “24 Week Schedule” >>
Description | <<A short description of the project requirement; for example: “The total of all (annual) costs of licenses for COTS products incorporated into the system shall not exceed $50,000“>>
Priority | <<The relative importance of this project requirement to the client organization; using the MSCW (MoSCoW) prioritization scheme: M (Must have), S (Should have), C (Could have), W (Want to have)>>
Win-Win Agreement(s) | <<The unique identifier(s) of the win-win agreement(s) from which this capability requirement arises>>


<br>

## Development Requirements

<br>

### Budget and Schedule

### Deployment Requirements

### Transition Requirements

### Support Environment Requirements

<br>

## Capability (Functional/Product) Requirements

* CR-1: The system shall enable both customers and system administrators to create new orders
* CR-2: The system shall enable both customers and system administrators to modify orders that have not yet been shipped
* CR-3: The system shall enable both customers and system administrators to delete orders that have not yet been shipped
* CR-4: The system shall enable customers, customer-relations personnel, and administrators to track orders, i.e., to determine:
o	If an order has been shipped
o	When an as yet un-shipped order is expected to be shipped, and the reason for delay in shipping if the order will not be shipped within the advertised window
o	Where a shipped, but not yet delivered, order is located, and the expected delivery date
* CR-5: The system shall take payment via Visa, MasterCard, American Express, and Diners Club
